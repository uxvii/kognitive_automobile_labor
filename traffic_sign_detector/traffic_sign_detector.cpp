#include "traffic_sign_detector.hpp"
#include <cv_bridge/cv_bridge.h>

//#include <mrt_cv_bridge_ros/cv_bridge.h>





namespace traffic_sign_detector_ros_tool {

TrafficSignDetector::TrafficSignDetector(ros::NodeHandle nhPublic, ros::NodeHandle nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate}, tfListener_{tfBuffer_} {

    /**
     * Initialization
     */
    interface_.fromParamServer();
    setupDiagnostics();
    pkgPath = ros::package::getPath("traffic_sign_detector_ros_tool");
    ROS_WARN_STREAM("path:  "<<pkgPath);

    winSize = 64;
    counter = 0;
    cv::HOGDescriptor hog2(cv::Size(winSize, winSize),
        cv::Size(32, 32),
        cv::Size(16, 16),
        cv::Size(32, 32),
        9, 1, -1, 0, 0.2,
        1, 64, 1);
    ROS_INFO_STREAM("HOG Descriptor created");
    imgColor = cv::Mat::zeros(1080, 1920, CV_8UC3);
    //imgDepth = cv::Mat::zeros(1080, 1920, CV_16UC1);
    svm = cv::ml::SVM::create();
    ROS_INFO_STREAM("Support Vector Machine Created");
    //////// TRAINING of SVM////////////
    if(!interface_.svmTrained) {
        trainStage(hog2, svm, trainImgs, trainLabels);
        svm->save(pkgPath + "/svm/svm.xml");
    }
    else{
    	svm = cv::ml::SVM::load(pkgPath + "/svm/svm.xml");
    	ROS_INFO_STREAM("Support Vector Machine loaded from file");
    }

    //////// TESTING OF SVM ///////////
    if(interface_.svmTest) {
    	testStage(hog2, svm, testImgs);
    }

    // A diagnosed pub can be used for message types with header.
    // This adds a diagnostics message for the frequency to this topic.
    // You can use the publisher in the interface object to create a diagnosed one from it.
    // publisherDiagnosed_ = std::make_unique<diagnostic_updater::DiagnosedPublisher<Msg>>(
    //     interface_.dummy_publisher, updater_,
    //     diagnostic_updater::FrequencyStatusParam(&interface_.diagnostic_updater_rate,
    //                                              &interface_.diagnostic_updater_rate,
    //                                              interface_.diagnostic_updater_rate_tolerance, 5),
    //     diagnostic_updater::TimeStampStatusParam());

    /**
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/TrafficSignDetector.if file.
     * Don't forget to register your callbacks here!
     */

    reconfigureServer_.setCallback(boost::bind(&TrafficSignDetector::reconfigureRequest, this, _1, _2));
    interface_.depth_subscriber->registerCallback(&TrafficSignDetector::depthCallback, this);
    interface_.image_subscriber->registerCallback(&TrafficSignDetector::callbackSubscriber, this);


    rosinterface_handler::showNodeInfo();
}

void TrafficSignDetector::callbackSubscriber(const Msg::ConstPtr& msg) {
	ROS_INFO_STREAM("NEW CALLBACK");
	cv::HOGDescriptor hog(cv::Size(winSize, winSize),			//nochmal überdenken wieso Werte aus Paper nicht gehen
	        cv::Size(32, 32),
	        cv::Size(16, 16),
	        cv::Size(32, 32),
	        9, 1, -1, 0, 0.2,
	        1, 64, 1);
	//traffic_sign_detector_ros_tool::Sign msgSign;
	std_msgs::Int32MultiArray msgSign;
	msgSign.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msgSign.layout.dim[0].size = 2;
	msgSign.layout.dim[0].stride = 1;
	msgSign.layout.dim[0].label = "1st type of traffic sign | 2nd distance of traffic sign";
	msgSign.data.resize(2);

	std::vector<cv::Mat> imgsMser;
	cv_bridge::CvImagePtr cv_ptr;

	// Convert from ROS Image msg to OpenCV image
	try {
	    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");//sensor_msgs::image_encodings::BGR8);
	    imgColor = cv_ptr->image;
	    //cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e) {
	    ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
	        msg->encoding.c_str());
	}

	//Main Algorithm
	if (!imgColor.empty()) {

		//Resize the image
		cv::resize(imgColor,imgColor,cv::Size(960,540));


		// Denoise image with gaussian blur
		auto start = std::chrono::high_resolution_clock::now();
		imgDenoise = this->deNoise(imgColor);
		auto end = std::chrono::high_resolution_clock::now();
		uint64_t duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
		ROS_WARN_STREAM("Time to denoise the image:  "<<duration);

		//normalize the image w.r.t. red and blue
		start = std::chrono::high_resolution_clock::now();
		imgRBNorm = this->preproc(imgDenoise);
		end = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
		ROS_WARN_STREAM("Time to preproc the image:  "<<duration);

	    // Get the detections using MSER
		start = std::chrono::high_resolution_clock::now();
		imgsMser = this->roiDetection(imgRBNorm, imgDenoise, area);
		end = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
		ROS_WARN_STREAM("Time to detect rois in the image:  "<<duration);
		ROS_INFO_STREAM("Number of MSER features:  "<<imgsMser.size());

	    // If there are detection in the frame:
	    if (imgsMser.size() != 0) {

	        // HOG features of detections
	    	start = std::chrono::high_resolution_clock::now();
	    	testHog = this->hogFeatures(hog, imgsMser);
	    	end = std::chrono::high_resolution_clock::now();
	    	duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
	    	ROS_WARN_STREAM("Time to calc HOG features:  "<<duration);

	        // Evaluate using the SVM
	    	start = std::chrono::high_resolution_clock::now();
	    	for(int i = 0; i < testHog.rows; i++){
	        	cv::Mat svmInput = testHog.row(i);
	        	this->traffic_sign = this->svmTesting(svm, svmInput);

	        	//saving the detection to folder
	        	if(interface_.saveDetection){
	        		this->saveDetection(imgsMser[i]);
	        	}
	        	// Publish the sign message if detection is really a traffic sign
	        	if(traffic_sign != 5) {
					this->distance = calcDistance(imgDepth,boxes[i]);
					ROS_INFO_STREAM("traffic sign:  "<<traffic_sign<<"    end");
					ROS_INFO_STREAM("distance:  "<<distance<<"  end");
					msgSign.data[0] = this->traffic_sign;
					msgSign.data[1] = this->distance;
					visualization(imgColor,this->boxes[i]);
					interface_.traffic_sign_publisher.publish(msgSign);		//doesnt work with rostopic echo wrong cmakefile
	        	}
	        }
	    	end = std::chrono::high_resolution_clock::now();
	    	duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
	    	ROS_WARN_STREAM("Time to evaluate the rois in the image:  "<<duration);
	    }
	    // Visualization of the robot view with all the detections
	    sensor_msgs::ImagePtr msgViz = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgColor).toImageMsg();
	    interface_.traffic_sign_viz_publisher.publish(msgViz);

	}


	imgsMser.clear();
	this->boxes.clear();
	msgSign.data.clear();

    // publisherDiagnosed_->publish(newMsg);

    // The updater will take care of publishing at a throttled rate
    // When calling update, all updater callbacks (defined in setupDiagnostics) will be run
    updater_.update();
}

void TrafficSignDetector::depthCallback(const Msg::ConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
	// Convert from ROS Image msg to OpenCV image
	try {
	    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);//sensor_msgs::image_encodings::BGR8);
	    this->imgDepth = cv_ptr->image;
	    //cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e) {
	    ROS_ERROR("Could not convert from '%s' to '16UC1'.",
	        msg->encoding.c_str());
	}

}

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void TrafficSignDetector::reconfigureRequest(const ReconfigureConfig& config, uint32_t level) {
    interface_.fromConfig(config);
    // If the publisherDiagnosed_ is configurable, you should update it here.
    // This introduces almost no overhead, no need to check if this is actually necessary.
    // publisherDiagnosed_ = std::make_unique<diagnostic_updater::DiagnosedPublisher<Msg>>(
    //     interface_.dummy_publisher, updater_,
    //     diagnostic_updater::FrequencyStatusParam(&interface_.diagnostic_updater_rate,
    //                                              &interface_.diagnostic_updater_rate,
    //                                              interface_.diagnostic_updater_rate_tolerance, 5),
    //     diagnostic_updater::TimeStampStatusParam());
}

/*
 * Setup the Diagnostic Updater
 */
void TrafficSignDetector::setupDiagnostics() {
    // Give a unique hardware id
    diagnosticStatus_.hardware_id = interface_.diagnostic_updater_hardware_id;
    diagnosticStatus_.message = "Starting...";
    diagnosticStatus_.level = diagnostic_msgs::DiagnosticStatus::STALE;
    updater_.setHardwareID(interface_.diagnostic_updater_hardware_id);

    // Add further callbacks (or unittests) that should be called regularly
    updater_.add("TrafficSignDetector Sensor Status", this, &TrafficSignDetector::checkSensorStatus);

    updater_.force_update();
}

void TrafficSignDetector::checkSensorStatus(diagnostic_updater::DiagnosticStatusWrapper& status_wrapper) {
    status_wrapper.summary(diagnosticStatus_);
    diagnosticStatus_.message = "Valid operation";
    diagnosticStatus_.level = diagnostic_msgs::DiagnosticStatus::OK;
}

//Classifier related methods
cv::Mat TrafficSignDetector::deNoise(cv::Mat input) {
    cv::Mat output;
    // Apply gaussian filter to denoise image
    cv::GaussianBlur(input, output, cv::Size(3, 3), 0, 0);
    //cv::namedWindow("Gaussian Blur",cv::WINDOW_NORMAL);
    //imshow("Gaussian Blur",output);
    return output;
}

cv::Mat TrafficSignDetector::preproc(cv::Mat input) {
	cv::Mat output;
	std::vector<cv::Mat> planes(3);
	cv::Mat imgF;
	cv::Mat rNormF;
	cv::Mat rNorm;
	cv::Mat bNormF;
	cv::Mat bNorm;

	input.convertTo(imgF, CV_32FC3);
	cv::split(imgF,planes);

	//red normalization
	cv::divide(planes[2],planes[0] + planes[1] + planes[2], rNormF);
	rNormF.convertTo(rNorm,CV_8UC1,255.0);

	//blue normalization
	cv::divide(planes[0],planes[0] + planes[1] + planes[2], bNormF);
	bNormF.convertTo(bNorm,CV_8UC1,255.0);

	//combine both
	max(rNorm,bNorm,output);

	//Plotting the normalized images
	//cv::namedWindow("red blue normalized image",cv::WINDOW_NORMAL);
	//imshow("red blue normalized image", output);

	return output;
}
std::vector<cv::Mat> TrafficSignDetector::roiDetection(cv::Mat input,cv::Mat raw, double &area) {
    cv::Mat detection;
    cv::Size size(winSize, winSize);
    std::vector<cv::Mat> detections;
    std::vector<std::vector<cv::Point> > regions;
    std::vector<cv::Rect> mserBoundBox;
    double top = interface_.ignoreTop*540.0;
    double height;
    cv::Rect rectsIntersection;
    cv::Scalar meanColor;
    bool addDetection;

    cv::Ptr<cv::MSER> ms = cv::MSER::create(interface_.delta, interface_.minArea, interface_.maxArea,
    		interface_.maxVariation, interface_.minDiversity);
    ms->detectRegions(input, regions, mserBoundBox);

    // For every bounding box in the image
    for (cv::Rect i : mserBoundBox) {

        double ratio = (static_cast<double>(i.height) / static_cast<double>(i.width));
        height = i.y + i.height;// + i.height;

        // Ratio filter and height filter of detected regions
        if ( interface_.minRatio < ratio && ratio < 1./interface_.minRatio && height > top) {

        	// Crop bounding boxes to get new images
            detection = raw(i);
            meanColor = cv::mean(detection);
            area = static_cast<double>(i.height) * static_cast<double>(i.width);

            //Color filter of detected regions
            if((meanColor[0] > meanColor[1] + interface_.blueGain*meanColor[0] && meanColor[0] > meanColor[2] + interface_.blueGain*meanColor[0])||
        		(meanColor[2] > meanColor[1] + interface_.redGain*meanColor[2] && meanColor[2] > meanColor[0] + interface_.redGain*meanColor[2])) {

            	// Resize images  to fit the trained data
				cv::resize(detection, detection, size);

				//Check if box contains other boxes
				addDetection = true;
				for (std::vector<cv::Rect>::iterator j = this->boxes.begin(); j != this->boxes.end();){

					rectsIntersection = i & *j;
					if(rectsIntersection.area() != 0){
						if(i.area() > j->area()){	//i is candidate to be added in boxes

							this->boxes.erase(j);
							detections.erase(detections.begin()+std::distance(this->boxes.begin(),j));
						}
						else{
							j++;
							addDetection = false;
						}
					}
					else{
						j++;
					}
				}
				if(addDetection){

					// Output the vector of images
					detections.push_back(detection);
					this->boxes.push_back(i);
				}
            }
        }
    }

    /**
     * Visualization of MSER detections
     */
 //   cv::Mat imgClone = raw.clone();
 //   for(std::vector<cv::Mat>::iterator it = detections.begin(); it != detections.end(); it++){
 //       cv::namedWindow("ROI Detectionnr: "+std::to_string(std::distance(detections.begin(),it)),cv::WINDOW_NORMAL);
 //       imshow("ROI Detectionnr: "+std::to_string(std::distance(detections.begin(),it)),*it);
 //   }

    return detections;
}

cv::Mat TrafficSignDetector::hogFeatures(cv::HOGDescriptor hog, std::vector<cv::Mat> imgs) {
    std::vector<std::vector<float> > HOG;

    // For all of the images of the vector, compute HOG features
    for (cv::Mat i : imgs) {
        std::vector<float> descriptor;
        hog.compute(i, descriptor);
        HOG.push_back(descriptor);
    }

    // Convert HOG features vector to Matrix for the SVM
    cv::Mat signMat(HOG.size(), HOG[0].size(), CV_32FC1);
    size_t i = 0;
    size_t j = 0;
    while (i < HOG.size()) {
        j = 0;
        while (j < HOG[0].size()) {
            signMat.at<float>(i, j) = HOG[i][j];
            j++;
        }
        i++;
    }
    HOG.clear();
	return signMat;
}

void TrafficSignDetector::loadTrainingImgs(std::vector<cv::Mat> &trainImgs,
    std::vector<int> &trainLabels) {
    // Load all the turn left signs images from dataset and label them
    cv::String pathname = pkgPath +"/Images/Images_GTSRB/Training/1";
    std::vector<cv::String> filenames;
    cv::glob(pathname, filenames);
    cv::Size size(winSize, winSize);

    for (cv::String i : filenames) {
        cv::Mat src = imread(i);

        cv::resize(src, src, size);
        trainImgs.push_back(src);
        trainLabels.push_back(1);
    }

    // Load all the turn right signs images from dataset and label them
    cv::String pathname2 = pkgPath + "/Images/Images_GTSRB/Training/2";
    std::vector<cv::String> filenames2;
    cv::glob(pathname2, filenames2);

    for (cv::String i : filenames2) {
        cv::Mat src2 = imread(i);

        cv::resize(src2, src2, size);
        trainImgs.push_back(src2);
        trainLabels.push_back(2);
    }

    // Load all the stop signs images from dataset and label them
    cv::String pathname3 = pkgPath + "/Images/Images_GTSRB/Training/3";
    std::vector<cv::String> filenames3;
    cv::glob(pathname3, filenames3);

    for (cv::String i : filenames3) {
        cv::Mat src3 = imread(i);

        cv::resize(src3, src3, size);
        trainImgs.push_back(src3);
        trainLabels.push_back(3);
    }

    // Load all the forward signs images from dataset and label them
    cv::String pathname4 = pkgPath + "/Images/Images_GTSRB/Training/4";
    std::vector<cv::String> filenames4;
    cv::glob(pathname4, filenames4);

    for (cv::String i : filenames4) {
        cv::Mat src4 = imread(i);

        cv::resize(src4, src4, size);
        trainImgs.push_back(src4);
        trainLabels.push_back(4);
    }


    // Load all the dont cares images from dataset and label them
    cv::String pathname5 = pkgPath + "/Images/Images_GTSRB/Training/5";
    std::vector<cv::String> filenames5;
    cv::glob(pathname5, filenames5);

    for (cv::String i : filenames5) {
        cv::Mat src5 = imread(i);

        cv::resize(src5, src5, size);
        trainImgs.push_back(src5);
        trainLabels.push_back(5);
    }

}

void TrafficSignDetector::loadTestingImgs(std::vector<cv::Mat> &testImgs) {
    // Load all the turn left signs images from dataset
    cv::String pathname = pkgPath +"/Images/Images_GTSRB/Testing";
    std::vector<cv::String> filenames;
    cv::glob(pathname, filenames);
    cv::Size size(winSize, winSize);

    for (cv::String i : filenames) {
        cv::Mat src = imread(i);

        cv::resize(src, src, size);
        testImgs.push_back(src);
    }

}

void TrafficSignDetector::svmTraining(cv::Ptr<cv::ml::SVM> &svm, cv::Mat trainHog,
    std::vector<int> trainLabels) {
    // Set parameters of the SVM
    svm->setGamma(0.50625);
    svm->setC(12.5);
    svm->setKernel(cv::ml::SVM::RBF);
    svm->setType(cv::ml::SVM::C_SVC);

    // Feed SVM with all the labeled data and train it
    cv::Ptr<cv::ml::TrainData> td = cv::ml::TrainData::create(trainHog,
        cv::ml::ROW_SAMPLE, trainLabels);
    svm->train(td);
}

void TrafficSignDetector::trainStage(cv::HOGDescriptor &hog, cv::Ptr<cv::ml::SVM> &svm,
    std::vector<cv::Mat> &trainImgs, std::vector<int> &trainLabels) {
    ROS_INFO_STREAM("SVM Training Stage started...");
    // Load training data and resize
    this->loadTrainingImgs(trainImgs, trainLabels);

    // HOG features of training images
    cv::Mat trainHOG = this->hogFeatures(hog, trainImgs);

    // Train SVM and save model
    this->svmTraining(svm, trainHOG, trainLabels);
    ROS_INFO_STREAM("SVM Training Stage completed");
    ros::Duration(1).sleep();

}

void TrafficSignDetector::testStage(cv::HOGDescriptor &hog, cv::Ptr<cv::ml::SVM> &svm,
    std::vector<cv::Mat> &testImgs) {


	ROS_INFO_STREAM("SVM Testing Stage started...");
	cv::Mat svmInput;
	cv::Mat imgViz;
	cv::Scalar green(0,255,0);
	int svmLabel;
	cv::Point org(5, 25);
	sensor_msgs::ImagePtr msgViz;
	// Load training data and resize
	this->loadTestingImgs(testImgs);

	// HOG features of testing images
    cv::Mat testHOG = this->hogFeatures(hog, testImgs);

    for(size_t i = 0; i < testImgs.size(); i++) {

		svmInput = testHOG.row(i);
		imgViz = testImgs[i];

		//Test SVM
		svmLabel = this->svmTesting(svm,svmInput);

		//Plot Input Img
		msgViz = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgViz).toImageMsg();
	    interface_.traffic_sign_viz_publisher.publish(msgViz);
	    ros::Duration(0.5).sleep();
	 	//Plot result
		//ROS_INFO_STREAM("Test result is  " << svmLabel);
		cv::resize(imgViz, imgViz, cv::Size(128,128) );
		switch(svmLabel){
		case 1 : cv::putText(imgViz, "Left", org, cv::FONT_HERSHEY_SIMPLEX, 0.8, green); break;
		case 2 : cv::putText(imgViz, "Right", org, cv::FONT_HERSHEY_SIMPLEX, 0.8, green); break;
		case 3 : cv::putText(imgViz, "Stop", org, cv::FONT_HERSHEY_SIMPLEX, 0.8, green); break;
		case 4 : cv::putText(imgViz, "Forward", org, cv::FONT_HERSHEY_SIMPLEX, 0.8, green); break;
		case 5 : cv::putText(imgViz, "Ignore", org, cv::FONT_HERSHEY_SIMPLEX, 0.8, green); break;
		default:;
		}
		msgViz = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgViz).toImageMsg();
	    interface_.traffic_sign_viz_publisher.publish(msgViz);
	    ros::Duration(0.5).sleep();


	}
    ROS_INFO_STREAM("SVM Test Stage completed");
}

int TrafficSignDetector::svmTesting(cv::Ptr<cv::ml::SVM> &svm, cv::Mat testHog) {
    cv::Mat answer;

    // Feed SVM with HOG features from detections and label it
    svm->predict(testHog, answer);

    // Return the label of the detection
    this->traffic_sign = answer.at<float>(0,0);
    this->traffic_sign = static_cast<int>(this->traffic_sign);
    return this->traffic_sign;

}

void TrafficSignDetector::visualization(cv::Mat &img, cv::Rect rect) {
	cv::Point org(rect.x, rect.y - 40);
	cv::Scalar color(0,255,0);
    // For all the signs in the image, print bounding box and type of sign
    switch(this->traffic_sign){
        case 1 : {
        	cv::rectangle(img, rect, color, 2);
        	cv::putText(img, "Left", org, cv::FONT_HERSHEY_DUPLEX, 1, color);
        	break;
        }
        case 2 : {
        	cv::rectangle(img, rect, color, 2);
        	cv::putText(img, "Right", org, cv::FONT_HERSHEY_DUPLEX, 1, color);
        	break;
        }
        case 3 : {
        	cv::rectangle(img, rect, color, 2);
        	cv::putText(img, "Stop", org, cv::FONT_HERSHEY_DUPLEX, 1, color);
        	break;
        }
        case 4 : {
        	cv::rectangle(img, rect, color, 2);
        	cv::putText(img, "Forward", org, cv::FONT_HERSHEY_DUPLEX, 1, color);
        	break;
        }
        case 5 : {
        	//cv::rectangle(img, rect, color, 2);
        	//cv::putText(img, "Ignore", org, cv::FONT_HERSHEY_DUPLEX, 1, color));
        	break;
        }
        default:;
    }
    //print distance to all signs
    if(this->traffic_sign!=5) {
    	std::string dist;
    	dist = boost::lexical_cast<std::string>(this->distance);
    	cv::putText(img, dist+" mm", org + cv::Point(0,30), cv::FONT_HERSHEY_DUPLEX, 1, color);
    }
}

void TrafficSignDetector::saveDetection(cv::Mat detection){
	std::string folder = pkgPath + "/Images/Detections/";
	std::string ppm = ".ppm";
	switch(this->traffic_sign){
		case 1: {
			imwrite(folder + "Left/" + std::to_string(counter) + ppm,detection);
			counter++;
			break;
		}
		case 2: {
			imwrite(folder + "Right/" + std::to_string(counter) + ppm,detection);
			counter++;
			break;
		}
		case 3: {
			imwrite(folder + "Stop/" + std::to_string(counter) + ppm,detection);
			counter++;
			break;
		}
		case 4: {
			imwrite(folder + "Forward/" + std::to_string(counter) + ppm,detection);
			counter++;
			break;
		}
		case 5: {
		    	imwrite(folder + "Ignore/" + std::to_string(counter) + ppm,detection);
		   	counter++;
		    	break;
		}
		default :;
	}
}

int TrafficSignDetector::calcDistance(cv::Mat &imgDepth, cv::Rect Box) {
	float distance=0;
	cv::Point begin;
	cv::Point end;
	cv::Mat roi;
	begin.x = Box.x + static_cast<int>(0.4* static_cast<double>(Box.width));
	begin.y = Box.y + static_cast<int>(0.4* static_cast<double>(Box.height));
	end.x = begin.x + static_cast<int>(0.2* static_cast<double>(Box.width));
	end.y = begin.y + static_cast<int>(0.2* static_cast<double>(Box.height));
	cv::Rect rectMean(begin,end);
	if(!imgDepth.empty()){
		cv::resize(imgDepth,imgDepth,cv::Size(960,540));
		roi = cv::Mat(imgDepth,rectMean);
		//calculate mean distance in rectMean
	    distance = 0;
	    for(cv::MatIterator_<ushort> it = roi.begin<ushort>(); it != roi.end<ushort>(); it++) {
	    	distance += static_cast<float>(*it);
	    }
	    distance = distance/static_cast<float>(roi.total());

	    //Plotting
	    cv::Mat plot = imgDepth.clone();
	    cv::rectangle(plot, rectMean, 10000, 2);
	    cv::namedWindow("Depth image",cv::WINDOW_NORMAL);
	    imshow("Depth image",plot);
	}



	return static_cast<int>(distance);
}
} // namespace traffic_sign_detector_ros_tool

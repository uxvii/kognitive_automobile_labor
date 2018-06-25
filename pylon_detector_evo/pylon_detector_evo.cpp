#include "pylon_detector_evo.hpp"

namespace demo_image_processing_ros_tool {

PylonDetectorEvo::PylonDetectorEvo(ros::NodeHandle nh_public, ros::NodeHandle nh_private)
        : params_{nh_private}, reconfigure_server_{nh_private}, image_transport_{nh_public} {

    /**
     * Initialization
     */
    utils_ros::setLoggerLevel(nh_private);
    params_.fromParamServer();

    /**
     * Publisher and subscriber setup
     */
    uv_publisher_ = nh_public.advertise<std_msgs::Int32MultiArray>(params_.uv_publisher,params_.msg_queue_size);
    publisher_ = image_transport_.advertiseCamera(params_.topic_publisher, params_.msg_queue_size);
    hsv_publisher_ = image_transport_.advertiseCamera(params_.hsv_publisher, params_.msg_queue_size);
    draw_publisher_ = image_transport_.advertiseCamera(params_.draw_publisher, params_.msg_queue_size);
    src_copy_publisher_ = image_transport_.advertiseCamera(params_.src_copy_publisher, params_.msg_queue_size);
    depth_publisher_ = image_transport_.advertiseCamera(params_.depth_publisher, params_.msg_queue_size);

    depth_subscriber_ = image_transport_.subscribeCamera(
		params_.depth_subscriber,
		params_.msg_queue_size,
		boost::bind(&PylonDetectorEvo::depthCallbackSubscriber, this, _1, _2));
    subscriber_ = image_transport_.subscribeCamera(
        params_.topic_subscriber,
        params_.msg_queue_size,
        boost::bind(&PylonDetectorEvo::callbackSubscriber, this, _1, _2));

    /**
     * Set up dynamic reconfiguration
     */
    reconfigure_server_.setCallback(boost::bind(&Parameters::fromConfig, &params_, _1, _2));

    /**
     * Display a summary after everything is set up
     */
    utils_ros::showNodeInfo();
}

void PylonDetectorEvo::callbackSubscriber(const sensor_msgs::Image::ConstPtr& msg,
                                          const sensor_msgs::CameraInfo::ConstPtr& camera_info) {

    /**
     * Convert ROS message using the cv_bridge
     */
	namespace enc = sensor_msgs::image_encodings;
    if (!enc::isColor(msg->encoding)) {
        ROS_ERROR_STREAM_THROTTLE(1, "Input is no color image");
        return;
    }

    cv_bridge::CvImage::ConstPtr cv_in;
    try {
        cv_in = cv_bridge::toCvShare(msg, enc::BGR8); // Source image
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR_STREAM(e.what());
        return;
    }
    const cv_bridge::CvImage::Ptr cv_hsv{new cv_bridge::CvImage(msg->header, enc::BGR8)}; // HSV image
    const cv_bridge::CvImage::Ptr cv_out{new cv_bridge::CvImage(msg->header, enc::MONO8)}; // Segmented binary image
	const cv_bridge::CvImage::Ptr src_copy{new cv_bridge::CvImage(msg->header, enc::BGR8)}; // Copy of source image
	src_copy->image = cv_in->image.clone();
	const cv_bridge::CvImage::Ptr drawing{new cv_bridge::CvImage(msg->header, enc::BGR8)}; // Contour image

    /**
     * Color segmentation algorithm
     * 1. Conversion to HSV color space
     * 2. Pick only a certain range of hue, saturation and range
     * 3. Reduce noise by smoothing the image (replace each pixel with the median of its neighboring pixels)
     * 4. Fill holes and reduce clutter by morphological operations
     */

	/// 1.
	cv::cvtColor(cv_in->image, cv_hsv->image, cv::COLOR_BGR2HSV);
	/// 2.
	cv::inRange(cv_hsv->image,
				cv::Scalar(params_.h_min, params_.s_min, params_.v_min),
				cv::Scalar(params_.h_max, params_.s_max, params_.v_max),
				cv_out->image);
	/// 3.
	cv::medianBlur(cv_out->image, cv_out->image, 2 * params_.median_blur_kernel_size - 1);
	/// 4.
	cv::Point anchor = cv::Point(-1,-1); // Anchor is at the center
	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
	cv::dilate(cv_out->image, cv_out->image, element, anchor, params_.iterations_dilate);
	cv::erode(cv_out->image, cv_out->image, element, anchor, params_.iterations_erode);

    /**
     * Pylon detection algorithm
     * 1. Find contours in the segmented binary image
     * 2. Calculate aspect ratio to sort out other objects with same color range
     * 3. Find convex hull of the set of points belonging to contours_ar
     * 4. Calculate center of gravity using image moments
     */

	/// 1.
	std::vector<std::vector<cv::Point>> contours; // Detected contours
	std::vector<cv::Vec4i> hierarchy; // Optional output vector, containing information about the image topology (vector with 4 dimensions, with each value an int)
	cv::findContours(cv_out->image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) ); // See paper
	/// 2.
	std::vector<cv::Rect> boundRect( contours.size() ); // Bounding rectangles around all detected contours
	std::vector<float> aspect_ratio( contours.size() );
	std::vector<std::vector<cv::Point>> contours_ar = {}; // Contours that hold the right aspect ratio (ar)
	std::vector<cv::Rect> boundRect_ar = {}; // Bounding rectangles around contours_ar
	for( size_t i = 0; i < contours.size(); i++ )
	{
		boundRect[i] = cv::boundingRect( cv::Mat(contours[i]) );
		aspect_ratio[i] = float(boundRect[i].width)/boundRect[i].height;
		if (aspect_ratio[i] < params_.aspect_ratio){
			contours_ar.push_back(contours[i]);
			boundRect_ar.push_back(boundRect[i]);
		}
	}
	/// 3.
	std::vector<std::vector<cv::Point>> hull( contours_ar.size() );
	for( size_t i = 0; i < contours_ar.size(); i++ )
		{   cv::convexHull( cv::Mat(contours_ar[i]), hull[i], false ); } // See paper
	/// 4.
	std::vector<cv::Moments> mu(contours_ar.size());
	for( size_t i = 0; i < contours_ar.size(); i++ ){
		mu[i] = cv::moments( contours_ar[i], false ); // Get the moments of a contour by using the Green's theorem
	}
	std::vector<cv::Point> mc( contours_ar.size() );
	for( size_t i = 0; i < contours_ar.size(); i++ ){
		if (mu[i].m00==0) {
			ROS_ERROR("Division through zero");
		}else{
		mc[i] = cv::Point( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
		}
	}

    /**
     * Get relevant points with their distance to anicar (100 pixels around the center of gravity)
     */
	std_msgs::Int32MultiArray msgPylon;
	cv::Point begin;
	cv::Point end;
	float dist = 0;
	for( size_t i = 0; i < contours_ar.size(); i++ ){
		if(!imgDepth.empty() && mc[i].x > 0 && mc[i].x < msg->width && mc[i].y > 0 && mc[i].y < msg->height){
			//100 pixels and every pixel with its own distance
			begin.x = mc[i].x - 5;
			begin.y = mc[i].y - 5;
			end.x = mc[i].x + 5;
			end.y = mc[i].y + 5;
			if(imgDepth.at<float>(begin.y,begin.x)!=0 && imgDepth.at<float>(end.y, end.x)!=0){
				cv::Rect rectMean(begin,end);
				for(size_t y = rectMean.y; y < rectMean.y + rectMean.height; y++) {
					for(size_t x = rectMean.x; x < rectMean.x + rectMean.width; x++) {
						dist = imgDepth.at<float>(y,x);
						msgPylon.data.push_back(x);
						msgPylon.data.push_back(y);
						msgPylon.data.push_back(dist);
					}
				}
			}
		}
	}

	/**
	 * Visualization
	 */
	// Make rectangle information global so that it can be used in depthCallbackSubscriber
	size = contours_ar.size();
	b.reserve(size);
	e.reserve(size);
	for( size_t i = 0; i < contours_ar.size(); i++ ){
		b[i].x = mc[i].x -5;
		b[i].y = mc[i].y -5;
		e[i].x = mc[i].x +5;
		e[i].y = mc[i].y +5;
	}
	cv::RNG& rng = cv::theRNG(); // RNG = random number generator
	drawing->image = cv::Mat::zeros(cv_out->image.size(), CV_8UC3 );
	int thickness = 10;
	int radius = 4;
	for( size_t i = 0; i< contours_ar.size(); i++ ){
		cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		cv::drawContours( drawing->image, hull, (int)i, color, thickness, 8, std::vector<cv::Vec4i>(), 0, cv::Point() ); // Draw convex hull
		cv::circle( drawing->image, mc[i], radius, color, thickness, 8, 0 ); // Draw center of gravity
		cv::rectangle( src_copy->image, boundRect_ar[i].tl(), boundRect_ar[i].br(), color, thickness, 8, 0 ); // Draw bounding box around pylon
	}

	/**
	 * Publish UV coordinates with depth info
	 */
	uv_publisher_.publish(msgPylon);
	/**
	 * Publish HSV image
	 */
	hsv_publisher_.publish(cv_hsv->toImageMsg(), camera_info);
    /**
     * Publish segmented image
     */
    publisher_.publish(cv_out->toImageMsg(), camera_info);
    /**
	 * Publish contour image
	 */
	draw_publisher_.publish(drawing->toImageMsg(), camera_info);
    /**
	 * Publish copy of source image
	 */
	src_copy_publisher_.publish(src_copy->toImageMsg(), camera_info);

}

void PylonDetectorEvo::depthCallbackSubscriber(const sensor_msgs::Image::ConstPtr& msg,
        									   const sensor_msgs::CameraInfo::ConstPtr& camera_info) {
    /**
     * Convert ROS message using the cv_bridge
     */
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1); // Depth image
		cv_ptr->image.convertTo(imgDepth,CV_32F);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to '16UC1'.",
			msg->encoding.c_str());
	}

	const cv_bridge::CvImage::Ptr depth_copy{new cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_16UC1)}; // Copy of depth image
	depth_copy->image = cv_ptr->image.clone();
	for( size_t i = 0; i< size; i++ ){
		cv::rectangle( depth_copy->image, b[i], e[i], (0,0,0), 2, 8, 0 ); // Draw rectangle from which the depth is taken as distance
	 }

    /**
	 * Publish depth image
	 */
	depth_publisher_.publish(depth_copy->toImageMsg(), camera_info);

}

}


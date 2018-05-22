#include "pylon_detector.hpp"

#include <cv_bridge/cv_bridge.h>
#include <motor_interface_ros_tool/Activation.h>
#include <motor_interface_ros_tool/MotorCommand.h>
#include <motor_interface_ros_tool/ServoCommand.h>
#include <utils_ros/ros_console.hpp>
#include <opencv2/core/version.hpp>





#include<std_msgs/Int32MultiArray.h>
#if CV_MAJOR_VERSION == 2
#include <opencv2/imgproc/imgproc.hpp>
#elif CV_MAJOR_VERSION == 3
#include <opencv2/imgproc.hpp>
#endif

namespace demo_image_processing_ros_tool {

PylonDetector::PylonDetector(ros::NodeHandle nh_public, ros::NodeHandle nh_private)
        : params_{nh_private}, reconfigure_server_{nh_private}, image_transport_{nh_public} {

    /**
     * Initialization
     */
    utils_ros::setLoggerLevel(nh_private);
    params_.fromParamServer();

    /**
     * Publisher and subscriber setup
     */



  uv_publisher_ = nh_public.advertise< std_msgs::Int32MultiArray>(params_.topic_pixel_position,params_.msg_queue_size);


    subscriber_ = image_transport_.subscribeCamera(
        params_.topic_subscriber,
        params_.msg_queue_size,
        boost::bind(&PylonDetector::callbackSubscriber, this, _1, _2));

    /**
     * Set up dynamic reconfiguration
     */
    reconfigure_server_.setCallback(boost::bind(&Parameters::fromConfig, &params_, _1, _2));

/**
     * Display a summary after everything is set up
     */
    utils_ros::showNodeInfo();
}




void PylonDetector::callbackSubscriber(const sensor_msgs::Image::ConstPtr& msg,
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
        cv_in = cv_bridge::toCvShare(msg, enc::BGR8);
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR_STREAM(e.what());
        return;
    }

    /**
lane detection
     */
///////////////////////main function/////////////////////////////////////////////////////////////
    cv::Mat src, src_gray;
    src = cv_in->image;

    int segments[4]={0,src.cols*4/8, src.cols*6/8,src.cols };
    cv::GaussianBlur(src, src, cv::Size(5, 5), 0, 0);

    cv::cvtColor(src,src_gray, cv::COLOR_BGR2GRAY);
    cv::Mat srcROI(src_gray, cv::Rect(0, src.rows / 2, src.cols , src.rows / 2));
    cv::threshold( srcROI ,srcROI, 200, 255, 3);
    //cv::adaptiveThreshold(srcROI ,srcROI,255,CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, 3, 5);
    cv::dilate(srcROI, srcROI, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8))); //to get more points


  //  cv::Canny(srcROI, srcROI, 20, 100);
    cv::Mat image_segment[3];
    std::vector<cv::Vec4i> lines[3];// 3 image and lines room
    for (size_t i = 0; i < 3; i++)
    {
        image_segment[i] = srcROI(cv::Rect(segments[i], 0, segments[i + 1] - segments[i], srcROI.rows));
    }                                                              //devided into 3 parts, each parts has own hough room



    int votethreshold[3] = { 20,15,20};
    int length_threshold[3]={50,10,50};


    std::vector<cv::Point> points[3];
    for (size_t i = 0; i < 3; i++)
    {
        cv::HoughLinesP(image_segment[i], lines[i], 1, CV_PI / 180, votethreshold[i], length_threshold[i], 100);    //find lines in each parts
        for (size_t j = 0; j < lines[i].size(); j++)
        {
            cv::Vec4i L= lines[i][j];

            if (abs(L[1] - L(3)) < 80)continue;   //if the slope of the line is too small, reject
            points[i].push_back(cv::Point(L[1], L[0]));
            points[i].push_back(cv::Point(L[3], L[2]));
            //points[i].push_bhttps://gitlab.mrt.uni-karlsruhe.de/pub/student_info/wikis/https://gitlab.mrt.uni-karlsruhe.de/pub/student_info/wikis/home(cv::Point(L[2], L[3]));
            cv::line(image_segment[i], cv::Point(L[0], L[1]), cv::Point(L[2], L[3]), 255, 3, CV_AA);

        }
        std::sort(points[i].begin(), points[i].end(), less_x);
        //cout << "next part" << endl;
        //cout << "all" << points[i] << endl;

    }
    cv::Mat P[3];
    for (int i = 0; i < 3; i++)
    {

        polynomial_curve_fit(points[i], 2, P[i]); // fitting curve
      //  cout << P[i] << endl;

    }

    std::vector<cv::Point> points_fitted[3];
    int x = 0;
    cv::Mat Imgline[3];
    int N;
    for (int i = 0; i < 3; i++)
    {
            Imgline[i] = image_segment[i] - image_segment[i];
            N = points[i].size()-1;
            if (N < 1) {
                points_fitted[i].push_back(cv::Point(0, 0));
                continue;
            }

            for (int j = points[i][0].x-1; j < srcROI.rows; j++)
            {


                    x= P[i].at<double>(0, 0) + P[i].at<double>(1, 0) * j +
                     P[i].at<double>(2, 0)*std::pow(j, 2);
                  if (x<0)x=0;
                  if (x>image_segment[i].cols)x=image_segment[i].cols;

                    points_fitted[i].push_back(cv::Point(x,j));    // based on the curve parameter


            }

            /*cv::polylines(image_segment[i], points_fitted[i], false, 255, 1, 8, 0);*/
            cv::polylines(Imgline[i], points_fitted[i], false, 255, 1, 8, 0);

    }

    for (size_t i = 0; i < 3; i++)
    {

        Output_points(points_fitted[i], segments[i], src.rows, i);
    }


    for (size_t i = 0; i < 3; i++)
    {
        Imgline[i].copyTo(image_segment[i]);
    }//https://gitlab.mrt.uni-karlsruhe.de/pub/student_info/wikis/home
    weight_change(P);
    finalpoints(points_fitted, src_gray);

    //cv::imshow("d",src_gray);




    std_msgs::Int32MultiArray Rospoints;
    for(std::vector<cv::Point>::const_iterator it = final_points.begin(); it != final_points.end(); ++it)
    {
        Rospoints.data.push_back(it->x);
        Rospoints.data.push_back(it->y);    //the  Rospoints vector?
    }


//cnvert the vector of uv to ros messages////////////////////////////////////////////////////////////////////////

  uv_publisher_.publish(Rospoints);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
}






//////////////////////////////////lane detection member function///////////////////////////////////////////////////////
bool PylonDetector::polynomial_curve_fit(std::vector<cv::Point>& key_point, int n, cv::Mat& A)
{
    //Number of key points
    int N = key_point.size();

    //contruction of matrix x
    cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_64FC1);//https://gitlab.mrt.uni-karlsruhe.de/pub/student_info/wikis/home
    for (int i = 0; i < n + 1; i++)
    {
        for (int j = 0; j < n + 1; j++)
        {
            for (int k = 0; k < N; k++)
            {
                X.at<double>(i, j) = X.at<double>(i, j) +
                    std::pow(key_point[k].x, i + j);
            }
        }
    }

    //the matrix Y
    cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_64FC1);
    for (int i = 0; i < n + 1; i++)
    {
        for (int k = 0; k < N; k++)
        {
            Y.at<double>(i, 0) = Y.at<double>(i, 0) +
                std::pow(key_point[k].x, i) * key_point[k].y;
        }
    }

    A = cv::Mat::zeros(n + 1, 1, CV_64FC1);
    //solveA
    cv::solve(X, Y, A, cv::DECOMP_LU);
    return true;
}
//transform into original image
void PylonDetector::Output_points(std::vector<cv::Point>& key_point,int width, int height,int i )
{
    int N = key_point.size();
    std::string K[3] = { "Left","Middle","Right" };
    std::sort(key_point.begin(),key_point.end(), more_y);
  //cout << K[i] << "Lane Points :" << endl;
    for (size_t j= 0; j < N; j++)
    {
        key_point[j].x = key_point[j].x + width;
        key_point[j].y = key_point[j].y + height/2;
        //cout << key_point[j].x << "x" << height - key_point[j].y << endl;
    }
}
//combine the information that could be found
bool PylonDetector::finalpoints(std::vector<cv::Point>key_point[3], cv::Mat &final)
{
    double scale[3];

    final_points.clear();

    if (key_point[1].size() < 2 )
    {
        if (key_point[2].size() > 2 && key_point[0].size() > 2)
        {
          scale[0] = key_point[0].size()/ M;
          scale[2] = key_point[2].size() / M;
            std::cout << "only provide right lane points" << std::endl;

            for (size_t i = 0; i < M-1; i++)
            {
                final_points.push_back(key_point[0][i*scale[0]] * 0.5  + key_point[2][i*scale[2]] * 0.5);
            }
        }
        else if (key_point[2].size() > 2)
        {
            scale[2] = key_point[2].size() / M;
            std::cout << "only provide left lane points" <<std:: endl;

            for (size_t i = 0; i < M; i++)
            {
                final_points.push_back(key_point[0][i*scale[2]]);
            }
        }

        else if(key_point[0].size() > 2)
        {
          scale[0] = key_point[0].size() / M;
          std::cout << "only provide left lane points" << std::endl;

          for (size_t i = 0; i < M; i++)
          {
              final_points.push_back(key_point[0][i*scale[0]]);
          }
        }


        else
        {
            std::cout << "no points could be provided" << std::endl;
            return true;
        }
    }
    else
    {
     if (key_point[1].size()<M )
      {
         M=key_point[1].size();

      }
     for (size_t i = 0; i < 3; i++)
     {
         scale[i] = key_point[i].size()/ M;
     }

     for (size_t i = 0; i < M-1; i++)
     {
         final_points.push_back(key_point[0][i*scale[0]] * weight[0] + key_point[1][i*scale[1]] * weight[1] + key_point[2][i*scale[2]] * weight[2]);
     }
     final_points.push_back(key_point[0][key_point[0].size() - 1] * weight[0] + key_point[1][key_point[1].size() - 1] * weight[1] + key_point[2][key_point[2].size() - 1] * weight[2]);
   }


    std::sort(final_points.begin(),final_points.end(),less_y);
    std::cout << "final points" << final_points <<std:: endl;
    for (int i = 0; i <M; i++)
    {
        cv::circle(final, final_points[i], 5, 255, 2, 8, 0);

    }
    return true;
}
//the weight of different line
void PylonDetector::weight_change(cv::Mat P[3])
{if(P[0].at<double>(2, 0)==0&&P[2].at<double>(2, 0)==0&&P[1].at<double>(2, 0)==0)

     {std::cout<<"no points could be provided"<<std::endl;}
  else if (P[0].at<double>(2, 0)==0||P[2].at<double>(2, 0)==0)
  {
      weight[0] = 0; weight[1] = 1; weight[2] = 0;
  }
  else if (abs(P[1].at<double>(2, 0)) > 0.06||abs(P[1].at<double>(2, 0))==0 )
  { if(abs(P[1].at<double>(2, 0)) <0.8)

      { weight[0] = 0.5; weight[1] = 0; weight[2] = 0.5; }
    else if(abs(P[1].at<double>(2, 0)) <2) { weight[0] = 0.45; weight[1] = 0.1; weight[2] = 0.45; }
    else{weight[0] = 0.1; weight[1] = 0.8; weight[2] = 0.1; }
  }
//    if (abs(P[0].at<double>(2, 0)) > 0.01||abs(P[2].at<double>(2, 0)) > 0.01){ weight[0] = 0; weight[1] = 1; weight[2] = 0; }
//
  else { weight[0] = 0; weight[1] = 1; weight[2] = 0; }
}










}

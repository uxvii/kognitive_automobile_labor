#pragma once

#include"opencv/cv.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <string>



#include <dynamic_reconfigure/server.h>
#include <image_transport/camera_publisher.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include <sensor_msgs/Image.h>

#include "demo_image_processing_ros_tool/PylonDetectorParameters.h"

namespace demo_image_processing_ros_tool {

class PylonDetector {

    using Parameters = PylonDetectorParameters;
    using ReconfigureServer = dynamic_reconfigure::Server<Parameters::Config>;

public:
    PylonDetector(ros::NodeHandle = ros::NodeHandle(), ros::NodeHandle = ros::NodeHandle("~"));

private:
    /**
     * This callback is called at startup or whenever a change was made in the dynamic_reconfigure
     * window
     * @param Received image
     */
    void callbackSubscriber(const sensor_msgs::Image::ConstPtr&,
                            const sensor_msgs::CameraInfoConstPtr&);



    Parameters params_; ///< Keeps all the parameters defined in "cfg/PylonDetector.mrtcfg"
    ReconfigureServer reconfigure_server_; ///< Enables us to change parameters during run-time

    image_transport::ImageTransport image_transport_; ///< Recommended image handle mechanism

    ros::Publisher uv_publisher_; ///< publishes commands to topic/uv
    image_transport::CameraSubscriber subscriber_;
    //ros::ServiceClient service_client_; ///< Lets us send a request to the motor interface




/////////////////////////////////////labne detection data member///////////////////////////////////////////////////////
    double weight[3] = { 0.25,0.5,0.25 };
                                        // the wight of left ,middle and right ;
    std::vector<cv::Point> final_points;
    int M=50  ;
/////////////////////////////////////labne detection member function///////////////////////////////////////////////////////
    static bool less_x(cv::Point m1, cv::Point m2) {
        return m1.x< m2.x;
    }
    static bool less_y(cv::Point m1, cv::Point m2) {
        return m1.y< m2.y;
    }
    static bool more_y(cv::Point m1, cv::Point m2) {
        return m1.y>m2.y;
    }

    bool polynomial_curve_fit(std::vector<cv::Point>& key_point, int n, cv::Mat& A);
    void Output_points(std::vector<cv::Point>& key_point, int width, int height, int i);
    bool finalpoints(std::vector<cv::Point>key_point[3], cv::Mat &final);
    inline void weight_change(cv::Mat P[3]);



};
}

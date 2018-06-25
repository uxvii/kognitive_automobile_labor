#pragma once

#include <dynamic_reconfigure/server.h>
#include <image_transport/camera_publisher.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include <utils_ros/ros_console.hpp>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32MultiArray.h>
#include <opencv2/core/version.hpp>
#include <math.h>
#include <cv_bridge/cv_bridge.h>
#if CV_MAJOR_VERSION == 2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#elif CV_MAJOR_VERSION == 3
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#endif

#include "demo_image_processing_ros_tool/PylonDetectorEvoParameters.h"

namespace demo_image_processing_ros_tool {

class PylonDetectorEvo {

    using Parameters = PylonDetectorEvoParameters;
    using ReconfigureServer = dynamic_reconfigure::Server<Parameters::Config>;

public:
    PylonDetectorEvo(ros::NodeHandle = ros::NodeHandle(), ros::NodeHandle = ros::NodeHandle("~"));

private:

    /**
     * Callbacks
     */
    void callbackSubscriber(const sensor_msgs::Image::ConstPtr&,
                            const sensor_msgs::CameraInfoConstPtr&);
    void depthCallbackSubscriber(const sensor_msgs::Image::ConstPtr&,
                                const sensor_msgs::CameraInfoConstPtr&);

    /**
	 * Publisher and subscriber
	 */
    Parameters params_; ///< Keeps all the parameters defined in "cfg/PylonDetector.mrtcfg"
    ReconfigureServer reconfigure_server_; ///< Enables us to change parameters during run-time
    image_transport::ImageTransport image_transport_; ///< Recommended image handle mechanism
    image_transport::CameraPublisher publisher_;      ///< Publishes the segmented image
    image_transport::CameraPublisher hsv_publisher_;      ///< Publishes the hsv image
    image_transport::CameraPublisher draw_publisher_;      ///< Publishes the edge image
    image_transport::CameraPublisher src_copy_publisher_;      ///< Publishes the src_copy image
    image_transport::CameraPublisher depth_publisher_;      ///< Publishes the depth image
    ros::Publisher uv_publisher_; ///< Publishes the u,v position of contour points

    image_transport::CameraSubscriber subscriber_;    ///< Subscribes to the color image
    image_transport::CameraSubscriber depth_subscriber_;    ///< Subscribes to the depth image

    /**
	 * Global variables
	 */
    cv::Mat imgDepth;
    size_t size;
	std::vector<cv::Point> b;
	std::vector<cv::Point> e;

};
}

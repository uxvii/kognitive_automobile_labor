#include "traffic_sign_detector.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "traffic_sign_detector_node");

    traffic_sign_detector_ros_tool::TrafficSignDetector traffic_sign_detector(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}

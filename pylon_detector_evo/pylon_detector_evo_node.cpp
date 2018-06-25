#include "pylon_detector_evo.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "pylon_detector_evo_node");

    demo_image_processing_ros_tool::PylonDetectorEvo pylon_detector_evo;

    ros::spin();
    return EXIT_SUCCESS;
}

#include "path_from_bag.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "path_from_bag_node");

    path_from_bag_ros_tool::PathFromBag path_from_bag(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}

#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "path_from_bag.hpp"

namespace path_from_bag_ros_tool {

class PathFromBagNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<PathFromBag>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<PathFromBag> impl_;
};
} // namespace path_from_bag_ros_tool

PLUGINLIB_EXPORT_CLASS(path_from_bag_ros_tool::PathFromBagNodelet, nodelet::Nodelet);

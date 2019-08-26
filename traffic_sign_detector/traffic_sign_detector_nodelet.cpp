#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "traffic_sign_detector.hpp"

namespace traffic_sign_detector_ros_tool {

class TrafficSignDetectorNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<TrafficSignDetector>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<TrafficSignDetector> impl_;
};
} // namespace traffic_sign_detector_ros_tool

PLUGINLIB_EXPORT_CLASS(traffic_sign_detector_ros_tool::TrafficSignDetectorNodelet, nodelet::Nodelet);

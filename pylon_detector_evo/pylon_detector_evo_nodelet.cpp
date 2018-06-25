#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "pylon_detector_evo.hpp"

namespace demo_image_processing_ros_tool {

class PylonDetectorEvoNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<PylonDetectorEvo>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<PylonDetectorEvo> impl_;
};
} // namespace demo_image_processing_ros_tool

PLUGINLIB_EXPORT_CLASS(demo_image_processing_ros_tool::PylonDetectorEvoNodelet, nodelet::Nodelet);

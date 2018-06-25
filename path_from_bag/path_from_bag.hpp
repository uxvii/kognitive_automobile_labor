#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <string.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>

#include "std_msgs/Int32MultiArray.h"

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/timer.h>


#include "path_from_bag_ros_tool/PathFromBagInterface.h"

namespace path_from_bag_ros_tool {

class PathFromBag {

    using Interface = PathFromBagInterface;
    using ReconfigureConfig = PathFromBagConfig;
    using ReconfigureServer = dynamic_reconfigure::Server<ReconfigureConfig>;

    using Msg = std_msgs::Header;

public:
    PathFromBag(ros::NodeHandle, ros::NodeHandle);

private:
  ////////////////////////////////////////////////////////////////
   void traffic_sign_callback(const std_msgs::Int32MultiArray::ConstPtr& msg);
    void callbackTimer(const ros::TimerEvent&);//check the state and acivate the read ros bag

    void read_bag( nav_msgs::Path::Ptr path_,std::string file_name );
  /////////////////////////////////////////////////////////////////////


    void reconfigureRequest(const ReconfigureConfig&, uint32_t);
    Interface interface_;
    ReconfigureServer reconfigureServer_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;


////////////////////////////////////////////////////////////////
    int state;  //1-16;
    //bool Stop=false;
    bool Turn_Right=false;
    bool Turn_Left=false;
    bool change_state=false;//95%

    ros::Timer timer_;// continus check state
    //std::map<double, Eigen::Affine3d> poses_;
    nav_msgs::Path::Ptr path_;
    std::vector<Eigen::Affine3d> path_eigen_;


    std::string pkgPath;

    nav_msgs::Path::Ptr path_1_{new nav_msgs::Path};
    nav_msgs::Path::Ptr path_2_{new nav_msgs::Path};
    nav_msgs::Path::Ptr path_3_{new nav_msgs::Path};
    nav_msgs::Path::Ptr path_4_{new nav_msgs::Path};
    nav_msgs::Path::Ptr path_5_{new nav_msgs::Path};
    nav_msgs::Path::Ptr path_6_{new nav_msgs::Path};
    nav_msgs::Path::Ptr path_7_{new nav_msgs::Path};
    nav_msgs::Path::Ptr path_8_{new nav_msgs::Path};
    nav_msgs::Path::Ptr path_9_{new nav_msgs::Path};
    nav_msgs::Path::Ptr path_10_{new nav_msgs::Path};
    nav_msgs::Path::Ptr path_11_{new nav_msgs::Path};
    nav_msgs::Path::Ptr path_12_{new nav_msgs::Path};
    nav_msgs::Path::Ptr path_13_{new nav_msgs::Path};
    nav_msgs::Path::Ptr path_14_{new nav_msgs::Path};
    nav_msgs::Path::Ptr path_15_{new nav_msgs::Path};
    nav_msgs::Path::Ptr path_16_{new nav_msgs::Path};
    nav_msgs::Path::Ptr path_17_{new nav_msgs::Path};


//////////////////////////////////////////////////////////////

};
} // namespace path_from_bag_ros_tool

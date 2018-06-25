#pragma once

#include <Eigen/Eigen>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/timer.h>
#include <tf2_ros/transform_listener.h>
#include "path_provider_ros_tool/PathProviderParameters.h"
#include <ros/service_client.h>

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <cmath>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"



#include <visualization_msgs/Marker.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

using namespace grid_map;
namespace path_provider_ros_tool {

class PathProvider {

    using Parameters = PathProviderParameters;
    using ReconfigureServer = dynamic_reconfigure::Server<Parameters::Config>;

public:
    PathProvider(ros::NodeHandle = ros::NodeHandle(), ros::NodeHandle = ros::NodeHandle("~"));

private:
    /**
     * This method is called at a fixed rate by the timer
     * @param TimerEvent structure
     */
    void callbackTimer(const ros::TimerEvent&);

    void callback_uv(const std_msgs::Int32MultiArray::ConstPtr& array);

    double signedAngleBetween(const Eigen::Vector3d& a, const Eigen::Vector3d& b);


    ros::Timer timer_;
    Parameters params_;                      ///< Keeps all relevant parameters, such as topic names, timer rate, etc.

    ReconfigureServer reconfigure_server_;   ///< Enables us to modify parameters during run-time
    tf2_ros::Buffer tf_buffer_;              ///< Buffers the transformation written to tf
    tf2_ros::TransformListener tf_listener_; ///< Manages tf_buffer object



    /**
     *publisher and subscribers of the class
     */
     ros::Publisher publisher_map_ ;
    ros::Publisher publisher_path_;          ///< Path publisher
     ros::Subscriber sub_uv_;



    /**
     *data members
     */

  //  std::map<double, Eigen::Affine3d> poses_;     //map  of  position    container       every element is tf_point_to_map_eigen
    nav_msgs::Path::Ptr path_{new nav_msgs::Path};

  Eigen::Vector2f f_forward;
  std::vector<int> array;


  ros::Publisher marker_pub;
  GridMap map_;





};

}

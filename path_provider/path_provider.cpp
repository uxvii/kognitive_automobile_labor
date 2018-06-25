#include "path_provider.hpp"
#include <eigen_conversions/eigen_msg.h>
#include <tf/tfMessage.h>
#include <utils_ros/ros_console.hpp>
#include <cmath>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/version.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/image_encodings.h>


#if CV_MAJOR_VERSION == 2
#include <opencv2/imgproc/imgproc.hpp>
#elif CV_MAJOR_VERSION == 3
#include <opencv2/imgproc.hpp>
#endif


#include <boost/algorithm/clamp.hpp>
#include <boost/math/special_functions/sign.hpp>
#include <boost/range/algorithm/min_element.hpp>



using namespace grid_map;

namespace path_provider_ros_tool {

PathProvider::PathProvider(ros::NodeHandle nh_public, ros::NodeHandle nh_private)
        : params_{nh_private}, reconfigure_server_{nh_private}, tf_listener_{tf_buffer_}{

    /**
     * Initialization
     */
    utils_ros::setLoggerLevel(nh_private);
    params_.fromParamServer();
    reconfigure_server_.setCallback(boost::bind(&Parameters::fromConfig, &params_, _1, _2));


//publisher and subscriber
    publisher_path_ = nh_private.advertise<nav_msgs::Path>(params_.topic_publisher, params_.msg_queue_size);


    sub_uv_         =  nh_public.subscribe(
        params_.topic_pixel_position, //the topic   params_.topic_pixel_position
        params_.msg_queue_size,
        &PathProvider::callback_uv,
        this
        );

    marker_pub = nh_public.advertise<visualization_msgs::Marker>(params_.marker, 1);
    publisher_map_ = nh_public.advertise<grid_map_msgs::GridMap>(params_.topic_grid_map, 1, true);

    double zero=0.0;
    map_.add("elevation",zero);
    map_.setFrameId("world");
    map_.setGeometry(Length(params_.map_size, params_.map_size), params_.map_resolotion, Position(params_.map_x, params_.map_y));

    timer_ = nh_private.createTimer(ros::Rate(params_.timer_rate), &PathProvider::callbackTimer, this);
    utils_ros::showNodeInfo();

}//end constructor




/**
       +++++++++++++++++++++++++++ get the u v z from another node ++++++++++++++++++++++++++++++++++++++++++++
                                  camera model
                                    update the map
     */
  void PathProvider::callback_uv(const std_msgs::Int32MultiArray::ConstPtr& array_u_v){

    this->array.clear();
    this->array=array_u_v->data;


    //do the preparation for the transform
    geometry_msgs::PoseStamped tf_point_to_camera_pose_ros; //point to camera positon in ros form
    geometry_msgs::PoseStamped tf_point_to_map_pose_ros;
   Eigen::Affine3d tf_point_to_camera_eigen;
   Eigen::Affine3d tf_point_to_map_eigen;

   Eigen::Affine3d tf_camera_to_map_eigen;
                        try {
                           ros::Time now = ros::Time::now();
                           const geometry_msgs::TransformStamped tf_camera_to_map_ros{tf_buffer_.lookupTransform(
                               params_.frame_id_map, params_.frame_id_camera, ros::Time(0), ros::Duration(3))};
                           tf::transformMsgToEigen(tf_camera_to_map_ros.transform, tf_camera_to_map_eigen);

                         } catch (const tf2::TransformException& e) {
                             ROS_WARN_STREAM(e.what());
                             return;
                         }



      // camera model to calculate point to map
        for(std::vector<int>::const_iterator it = this-> array.begin(); it != this->array.end(); it=it+3)
        {


                               int u=*it;
                               int v=*(it+1);
                               double z=*(it+2)/1000.0;// depth in m

                               float x=(u - params_.cx) * z / params_.fx;
                               float y=(v - params_.cy) * z / params_.fy;
                               //std::cout<<"x"<<" "<<x<<"z"<<" "<<z<<std::endl;

                                if( z>0 && z< params_.camera_depth_range)
                                  {
                                    // have to transform  xyz to world xyz

                                    tf_point_to_camera_pose_ros.pose.position.x=x;
                                    tf_point_to_camera_pose_ros.pose.position.y=0.2;
                                    tf_point_to_camera_pose_ros.pose.position.z=z;
                                    tf_point_to_camera_pose_ros.pose.orientation.x =0;
                                    tf_point_to_camera_pose_ros.pose.orientation.y =0;
                                    tf_point_to_camera_pose_ros.pose.orientation.z =0;
                                    tf_point_to_camera_pose_ros.pose.orientation.w =0;

                                    tf::poseMsgToEigen(tf_point_to_camera_pose_ros.pose, tf_point_to_camera_eigen);//ros pos  to eigen
                                    tf_point_to_map_eigen=tf_camera_to_map_eigen*tf_point_to_camera_eigen;
                                    tf::poseEigenToMsg(tf_point_to_map_eigen, tf_point_to_map_pose_ros.pose);
                                  //  map operation

                                  Position position(tf_point_to_map_pose_ros.pose.position.x, tf_point_to_map_pose_ros.pose.position.y);
                                //  std::cout<<"x"<<" "<<position.x()<<" "<<"y"<<" "<<position.y()<<std::endl;


                                  float tmp=map_.atPosition("elevation", position);
                                  if(tmp>0 ||tmp==0)
                                    {;}
                                  else
                                    {map_.atPosition("elevation", position)=0.0;
                                      tmp=0.0;}

                                  map_.atPosition("elevation", position) =params_.occupy_prob+tmp;
                                //  std::cout<<map_.atPosition("elevation", position)<<std::endl;
                                  }//end if
           }//for loop



}//end callback uvz





/**
                                    calculate the vehicle heading vector in world frame
                                    get the vehicle position
       +++++++++++++++++++++++++++ iterate through the map ,get the grid position > 0.9 ++++++++++++++++++++++++++++++++++++++++++++
                                   for   calculate the force
                                   generate the final force
                                   set the force with -45 to 45 degrees
                                   generate the path

     */

void PathProvider::callbackTimer(const ros::TimerEvent& timer_event) {
  path_->poses.clear();
  path_->header.frame_id = params_.frame_id_map;

  /////////////////visualisation in rviz/////////////////////////////
    visualization_msgs::Marker sphere_list;
    sphere_list.header.frame_id= params_.frame_id_map;
     sphere_list.header.stamp= ros::Time::now();
     sphere_list.ns= "spheres";
     sphere_list.action= visualization_msgs::Marker::ADD;
     sphere_list.pose.orientation.w= 1.0;

     sphere_list.id = 0;

     sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;


     // POINTS markers use x and y scale for width/height respectively
     sphere_list.scale.x = 0.1;
     sphere_list.scale.y = 0.1;
     sphere_list.scale.z = 0.1;

     // Points are green
     sphere_list.color.r = 1.0f;
     sphere_list.color.a = 1.0;

    ///////////////////////////////////////////////////////////



// get the position of the car in the map
Eigen::Affine3d vehicle_pose;
float x_vehicle=0;
float y_vehicle=0;

        try {
            const geometry_msgs::TransformStamped tf_ros =
                tf_buffer_.lookupTransform(params_.frame_id_map, params_.frame_id_camera, ros::Time(0));
            vehicle_pose = tf2::transformToEigen(tf_ros);
        } catch (const tf2::TransformException& e) {
            ROS_WARN_STREAM(e.what());
            return;
        }
 Eigen::Vector3d vehicle_position = vehicle_pose.translation();

 Position position_vehicle_for_map(vehicle_position(0),vehicle_position(1));

 //vehicle heading vector in world frame
 Eigen::Vector3d vehicle_unit_heading = [&vehicle_pose]() {
        Eigen::Vector3d p = vehicle_pose.rotation() * Eigen::Vector3d::UnitZ();
        p.z() = 0.0;
        return p.normalized();
    }();






  Eigen::Vector3d vehicle_unit_right = [&vehicle_pose]() {
           Eigen::Vector3d p = vehicle_pose.rotation() * Eigen::Vector3d::UnitX();
           p.z() = 0.0;
           return p.normalized();
       }();







//shift the vehicle point towards
// this is designed so that the points ahead hat bigger impacts than the points behind
position_vehicle_for_map.x()=position_vehicle_for_map.x()+vehicle_unit_heading.x()*params_.ahead_offset;
position_vehicle_for_map.y()=position_vehicle_for_map.y()+vehicle_unit_heading.y()*params_.ahead_offset;
//
float ir_origin_x=position_vehicle_for_map.x()-vehicle_unit_heading.x()*params_.ahead_offset;
float ir_origin_y=position_vehicle_for_map.y()-vehicle_unit_heading.y()*params_.ahead_offset;







//shift the vehicle point right
// this is designed so that the points ahead hat bigger impacts than the points behind
float right_x=ir_origin_x+vehicle_unit_right.x()*params_.side_offset;
float right_y=ir_origin_y+vehicle_unit_right.y()*params_.side_offset;
//




//shift the vehicle point left
// this is designed so that the points ahead hat bigger impacts than the points behind
float left_x=ir_origin_x-vehicle_unit_right.x()*params_.side_offset;
float left_y=ir_origin_y-vehicle_unit_right.y()*params_.side_offset;
//






 f_forward.x()=vehicle_unit_heading.x()*params_.k_forward_force;
 f_forward.y()=vehicle_unit_heading.y()*params_.k_forward_force;
 Eigen::Vector2f F_final=f_forward;


 //iterate through the map
 for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
      Position position_occupy;
      //the the position of occupy grid
      if(map_.at("elevation", *it)>params_.occupy_prob*params_.k_occupy_prob)
            {
                map_.getPosition(*it, position_occupy);
                //calculate the force between vehicle and occu_grid
                //update the force_final
                x_vehicle=position_vehicle_for_map.x();  //shifted head
                y_vehicle=position_vehicle_for_map.y();




                float x_grid=position_occupy.x();
                float y_grid=position_occupy.y();


                ////////////////////visualisation marker///////////////////////
                geometry_msgs::Point p;
                p.x = x_grid;
                p.y = y_grid;
                p.z = 0.0;
                sphere_list.points.push_back(p);


                //calculate the force  to the ahead point
                Eigen::Vector2f f_pylon;

                float distance=pow(  ( pow(x_vehicle-x_grid,2)+pow(y_vehicle-y_grid,2))      ,      params_.dis_power    );
                if (distance<params_.force_range && distance>0 )
                      {
                        f_pylon(0)=params_.k_pylon_force* (1/distance-1/params_.force_range)*(1/distance/distance)*(x_vehicle-x_grid)/distance;
                        f_pylon(1)=params_.k_pylon_force*  (1/distance-1/params_.force_range)*(1/distance/distance)*(y_vehicle-y_grid)/distance;
                        F_final=F_final+f_pylon;
                      }//end if




                  //calculate the force  to the left side point
                      float distance_left=pow(  ( pow(left_x-x_grid,2)+pow(left_y-y_grid,2))      ,      params_.dis_power    );
                        if (distance_left<params_.force_range_side && distance_left>0 )
                                        {
                                          f_pylon(0)=params_.k_pylon_force_side* (1/distance_left-1/params_.force_range_side)*(1/distance_left/distance_left)*(ir_origin_x-x_grid)/distance_left;
                                          f_pylon(1)=params_.k_pylon_force_side*  (1/distance_left-1/params_.force_range_side)*(1/distance_left/distance_left)*(ir_origin_y-y_grid)/distance_left;
                                          F_final=F_final+f_pylon;
                                        }//end if
                  //calculate the force  to the right side point
                  float distance_right=pow(  ( pow(right_x-x_grid,2)+pow(right_y-y_grid,2))      ,      params_.dis_power    );
                    if (distance_right<params_.force_range_side && distance_right>0 )
                                    {
                                      f_pylon(0)=params_.k_pylon_force_side* (1/distance_right-1/params_.force_range_side)*(1/distance_right/distance_right)*(ir_origin_x-x_grid)/distance_right;
                                      f_pylon(1)=params_.k_pylon_force_side*  (1/distance_right-1/params_.force_range_side)*(1/distance_right/distance_right)*(ir_origin_y-y_grid)/distance_right;
                                      F_final=F_final+f_pylon;
                                    }//end if







            }//end if

}//end for

////////////////////visualisation marker///////////////////////
                 marker_pub.publish(sphere_list);
//visualisation of grid map////////////////////////
ros::Time time = ros::Time::now();
map_.setTimestamp(time.toNSec());
grid_map_msgs::GridMap message;
GridMapRosConverter::toMessage(map_, message);
publisher_map_.publish(message);

////////////generate fictions points and path////////////////////////////////////////////////////



////restrict the force with the angle range -45 and 45 degree//////////////////////////
//vehicle 45 left
 Eigen::Vector3d left_45;// vehicle frame
 left_45(0)=-10; //x
  left_45(1)=0.3;//y
   left_45(2)=20;//z
Eigen::Vector3d vehicle_45_left = [&vehicle_pose,&left_45]() {
       Eigen::Vector3d p = vehicle_pose.rotation() * left_45;
       p.z() = 0.0;
       return p.normalized();
   }();
//vehilce 45 right
Eigen::Vector3d right_45;   //arctan(1.5)
right_45(0)=10; //x
 right_45(1)=0.3;//y
  right_45(2)=20;//z
Eigen::Vector3d vehicle_45_right = [&vehicle_pose,&right_45]() {
      Eigen::Vector3d p = vehicle_pose.rotation() * right_45;
      p.z() = 0.0;
      return p.normalized();
  }();



  //F_final  Eigen::Vector2f
  //vehicle_unit_heading  Eigen::Vector3d
//  Vector3d vehicle_45_left
//Vector3d vehicle_45_right
Eigen::Vector3d F_final_3d;
F_final_3d(0)=F_final(0);
F_final_3d(1)=F_final(1);
F_final_3d(2)=0;  //z


double delta_angle = signedAngleBetween(F_final_3d, vehicle_unit_heading);





if(delta_angle>3.1415926535/6.0) //pi/6
    F_final_3d=vehicle_45_right;

if(delta_angle<-3.1415926535/6.0) //pi/6
    F_final_3d=vehicle_45_left;

///////////////////////////////////////////////////////F_final_3d////////////7777

F_final_3d=F_final_3d.normalized();

x_vehicle=x_vehicle-vehicle_unit_heading.x()*params_.ahead_offset;
y_vehicle=y_vehicle-vehicle_unit_heading.y()*params_.ahead_offset;


  for(int i=-50;i<1500;i++)
                {
                            geometry_msgs::PoseStamped tf_point_to_map_pose_ros;

                            float x= F_final_3d(0)/1000*i;
                            float y=F_final_3d(1)/1000*i;
                            //add x y to vehicle pose
                            tf_point_to_map_pose_ros.pose.position.x=x+x_vehicle;  // x_vehicle = shifted vehicle head
                            tf_point_to_map_pose_ros.pose.position.y=y+y_vehicle;
                            tf_point_to_map_pose_ros.pose.position.z=0;
                           path_->poses.emplace_back(tf_point_to_map_pose_ros);

                }//end for


    path_->header.stamp = timer_event.current_expected;
    publisher_path_.publish(path_);

}//end call back timer
////////////////////////////////////////////////////////////////////////////////////////////


double PathProvider::signedAngleBetween(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    const double vz = boost::math::sign(a.cross(b).z());
    return vz * std::acos(a.normalized().dot(b.normalized()));
}
//////////////////////////////////////////////////////////////////////////////////////////////////////


}//end namespace

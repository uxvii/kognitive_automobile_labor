#include "path_from_bag.hpp"

#include <eigen_conversions/eigen_msg.h>
#include <tf/tfMessage.h>
//#include <utils_ros/ros_console.hpp>

#include "bag.hpp"
#include <tf2_eigen/tf2_eigen.h>
//#include <tf2_geometry_msgs.h>
#include <boost/range/algorithm/min_element.hpp>
#include <boost/math/special_functions/sign.hpp>
#include "safe_iterator_operations.h"
#include <ros/package.h>
#include <cmath>

namespace path_from_bag_ros_tool {

PathFromBag::PathFromBag(ros::NodeHandle nhPublic, ros::NodeHandle nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate}, tfListener_{tfBuffer_} {

    /**
     * Initialization
     */
    interface_.fromParamServer();


    /**
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/PathFromBag.if file.
     * Don't forget to register your callbacks here!
     */
    reconfigureServer_.setCallback(boost::bind(&PathFromBag::reconfigureRequest, this, _1, _2));
    timer_ = nhPrivate.createTimer(ros::Rate(interface_.timer_rate), &PathFromBag::callbackTimer, this);



    ///////////////////////////////////////////////////////////////////////////////////////////////
     pkgPath = ros::package::getPath("path_from_bag_ros_tool");


    read_bag(path_1_,pkgPath+"/launch/path_bag/t1.bag");
   read_bag(path_2_,pkgPath+"/launch/path_bag/t2.bag");
    read_bag(path_3_,pkgPath+"/launch/path_bag/t3.bag");
    read_bag(path_4_,pkgPath+"/launch/path_bag/t4.bag");

    read_bag(path_5_,pkgPath+"/launch/path_bag/t5.bag");

    read_bag(path_6_,pkgPath+"/launch/path_bag/t6.bag");
    read_bag(path_7_,pkgPath+"/launch/path_bag/t7.bag");
    read_bag(path_8_,pkgPath+"/launch/path_bag/t8.bag");
    read_bag(path_9_,pkgPath+"/launch/path_bag/t9.bag");
    read_bag(path_11_,pkgPath+"/launch/path_bag/t11.bag");
    read_bag(path_12_,pkgPath+"/launch/path_bag/t12.bag");
    read_bag(path_13_,pkgPath+"/launch/path_bag/t13.bag");

    read_bag(path_14_,pkgPath+"/launch/path_bag/t14.bag");
    read_bag(path_10_,pkgPath+"/launch/path_bag/t10.bag");

    read_bag(path_15_,pkgPath+"/launch/path_bag/t15.bag");

    read_bag(path_16_,pkgPath+"/launch/path_bag/t16.bag");
    read_bag(path_17_,pkgPath+"/launch/path_bag/t17.bag");

    interface_.traffic_sign_subscriber->registerCallback(&PathFromBag::traffic_sign_callback, this);



    rosinterface_handler::showNodeInfo();
this->state=0;

}

///////////////////////////////////////////////////////////////////////////////

void PathFromBag::traffic_sign_callback(const std_msgs::Int32MultiArray::ConstPtr& msg) {//   react to the traffic signs
    if( msg->data[0] == 1)
        this->Turn_Left=true;
    if(msg->data[0]  == 2)
        this->Turn_Right=true;


}


void PathFromBag::callbackTimer(const ros::TimerEvent&){//check the state and acivate the read ros bag

 if(this->state==0){



//++++++++++++++++++++++++++++prepare  the point of z 500+++++++++++++++++++++++++
//                              tf_point_to_map_pose_ros.pose.position.x  / y

		   geometry_msgs::PoseStamped tf_point_to_camera_pose_ros; //point to camera positon in ros form
		   geometry_msgs::PoseStamped tf_point_to_map_pose_ros;
		   Eigen::Affine3d tf_camera_to_map_eigen;
		  Eigen::Affine3d tf_point_to_camera_eigen; //point to cam in eigen form
		  Eigen::Affine3d tf_point_to_map_eigen;  //point to map

		  try {
				   ros::Time now = ros::Time::now();
				   const geometry_msgs::TransformStamped tf_camera_to_map_ros{tfBuffer_.lookupTransform(
				       interface_.frame_id_map, interface_.frame_id_ir, ros::Time(0), ros::Duration(3))};
				   tf::transformMsgToEigen(tf_camera_to_map_ros.transform, tf_camera_to_map_eigen);

				 } catch (const tf2::TransformException& e) {
				     ROS_WARN_STREAM(e.what());
				     return;
				 }

		  tf_point_to_camera_pose_ros.pose.position.x=0;
		  tf_point_to_camera_pose_ros.pose.position.y=0.2;
		  tf_point_to_camera_pose_ros.pose.position.z=500;
		 tf_point_to_camera_pose_ros.pose.orientation.x =0;
		  tf_point_to_camera_pose_ros.pose.orientation.y =0;
		  tf_point_to_camera_pose_ros.pose.orientation.z =0;
		  tf_point_to_camera_pose_ros.pose.orientation.w =0;

		      tf::poseMsgToEigen(tf_point_to_camera_pose_ros.pose, tf_point_to_camera_eigen);//ros pos  to eigen
		      tf_point_to_map_eigen=tf_camera_to_map_eigen*tf_point_to_camera_eigen;
		      tf::poseEigenToMsg(tf_point_to_map_eigen, tf_point_to_map_pose_ros.pose);
//////////////////

//++++++++++++++++++++++++++++prepare  6 starting points +++++++++++++++++++++++++
Eigen::Vector2f p1; p1.x()= ; p1.y()= ;
Eigen::Vector2f p2; p2.x()= ; p2.y()= ;
Eigen::Vector2f p3; p3.x()= ; p3.y()= ;
Eigen::Vector2f p4; p4.x()= ; p4.y()= ;
Eigen::Vector2f p5; p5.x()= ; p5.y()= ;
Eigen::Vector2f p6; p6.x()= ; p6.y()= ;
///////

//++++++++++++++++++++++++++++get the pose of the car +++++++++++++++++++++++++
Eigen::Vector3d vehicle_position = tf_camera_to_map_eigen.translation();
///////

//++++++++++++++++++++++++++++cal the distance     find the index of the smallest distance+++++++++++++++++++++++++
std::vector<double> distance;
double d1=pow(  ( pow(vehicle_position.x()-p1.x(),2)+pow(vehicle_position.y()-p1.y(),2))      ,      0.5   );
distance.push_back(d1);
double d2=pow(  ( pow(vehicle_position.x()-p2.x(),2)+pow(vehicle_position.y()-p2.y(),2))      ,      0.5   );
distance.push_back(d2);
double d3=pow(  ( pow(vehicle_position.x()-p3.x(),2)+pow(vehicle_position.y()-p3.y(),2))      ,      0.5   );
distance.push_back(d3);
double d4=pow(  ( pow(vehicle_position.x()-p4.x(),2)+pow(vehicle_position.y()-p4.y(),2))      ,      0.5   );
distance.push_back(d4);
double d5=pow(  ( pow(vehicle_position.x()-p5.x(),2)+pow(vehicle_position.y()-p5.y(),2))      ,      0.5   );
distance.push_back(d5);
double d6=pow(  ( pow(vehicle_position.x()-p6.x(),2)+pow(vehicle_position.y()-p6.y(),2))      ,      0.5   );
distance.push_back(d6);


int minPos = 0;
for (unsigned i = 0; i < distance.size(); ++i)
    {
        if (distance[i] < distance[minPos]) // Found a smaller min
            minPos = i;
    }


//++++++++++++++++++++++++++++switch case +++++++++++++++++++++++++
switch(minPos){
      case(0):{   if(tf_point_to_map_pose_ros.pose.position.x >0)
                      { this->state=13;   ROS_INFO("fuck you");system("rosrun dynamic_reconfigure dynparam set longitudinal_controller velocity 0.5");}

                  else
               		 	{this->state=14;system("rosrun dynamic_reconfigure dynparam set longitudinal_controller velocity 0.8");}
          break;}//end case 0

      case(2):{   if(tf_point_to_map_pose_ros.pose.position.x >0)
                          { this->state=14;  }

                  else
                   		 	{this->state=13;}
          break;}//end case 0

      case(3):{   if(tf_point_to_map_pose_ros.pose.position.x >0)
                              { this->state=15;  }

                  else
                            {this->state=16;}
          break;}//end case 0

      case(5):{   if(tf_point_to_map_pose_ros.pose.position.x >0)
                                  { this->state=16;  }

                  else
                                {this->state=15;}
          break;}//end case 0


      case(1):{   if(tf_point_to_map_pose_ros.pose.position.y >0)
                                      { this->state=14;  }

                  else
                                    {this->state=13;}
              break;}//end case 0

        case(4):{   if(tf_point_to_map_pose_ros.pose.position.y >0)
                                              { this->state=16;  }

                    else
                                            {this->state=15;}
                break;}//end case 0

   }//end switch


//////////////////////////////////////////////////////////////////////////

		switch(state){
		      case(1):path_=path_1_;break;
		      case(2):path_=path_2_;break;
		      case(3):path_=path_3_;break;
		      case(4):path_=path_4_;break;
		      case(5):path_=path_5_;break;
		      case(6):path_=path_6_;break;
		      case(7):path_=path_7_;break;
		      case(8):path_=path_8_;break;
		      case(9):path_=path_9_;break;
		      case(10):path_=path_10_;break;
		      case(11):path_=path_11_;break;
		      case(12):path_=path_12_;break;
		      case(13):path_=path_13_;break;
		      case(14):path_=path_14_;break;
		      case(15):path_=path_15_;break;
		      case(16):path_=path_16_;break;

    			 }//end switch
	return;
	}//end if



    this->change_state=false;
    interface_.path_publisher.publish(path_);
    //////check if i have arrived the 95%place
    /////copy the pose to Eigen
    path_eigen_.clear();
    path_eigen_.reserve(path_->poses.size());
    for (const auto& pose_stamped : path_->poses){
            Eigen::Affine3d pose;
            tf2::fromMsg(pose_stamped.pose, pose);
            path_eigen_.push_back(pose);
        }
        /*
         * Lookup the latest transform from vehicle to map frame
         */
    Eigen::Affine3d vehicle_pose;
        try {
            const geometry_msgs::TransformStamped tf_ros =
                tfBuffer_.lookupTransform(interface_.frame_id_map, interface_.vehicle_frame_id, ros::Time(0));
            vehicle_pose = tf2::transformToEigen(tf_ros);
        } catch (const tf2::TransformException& e) {
            ROS_WARN_STREAM(e.what());
            return;
        }

    const Eigen::Vector3d vehicle_position = vehicle_pose.translation();
    /*
     * Lookup stringthe closest point to the vehicle    it
     */
     auto const& it = boost::range::min_element(
        path_eigen_, [&vehicle_position](const Eigen::Affine3d& lhs, const Eigen::Affine3d& rhs) {
            return (lhs.translation() - vehicle_position).squaredNorm() <
                   (rhs.translation() - vehicle_position).squaredNorm();
        });
        /*
         * get the distance of it from the begin   95%??????
         */
    double dis=std::distance(path_eigen_.begin(), it);
    double length=path_eigen_.size();

    this->change_state=(dis/length)>interface_.percentage;

    if((dis/length)<interface_.percentage_forget)
      {this->Turn_Left=false;
      this->Turn_Right=false;}
/////////////////////////////////////////////////////////////////////////////////////7
        /*
         * change state???????
         */
    if(change_state==true){
                    switch(state){                        //change state
                      case(1):path_=path_16_;this->state=16;Turn_Left=false;Turn_Right=false;break;
                      case(2):
                      //#######################
                      {

                        if(this->Turn_Right==true)
                            {path_=path_17_;this->state=17;Turn_Right=false;}
                        else
                            {path_=path_14_;this->state=14;}
                        break;

                      }
              //############################3
                      case(3):path_=path_15_;this->state=15;Turn_Left=false;Turn_Right=false;break;
                      case(4):path_=path_13_;this->state=13;Turn_Left=false;Turn_Right=false;break;
                      case(5):path_=path_16_;this->state=16;Turn_Left=false;Turn_Right=false;break;
                      case(6):path_=path_13_;this->state=13;Turn_Left=false;Turn_Right=false;break;
                      case(7):path_=path_15_;this->state=15;Turn_Left=false;Turn_Right=false;break;
                      case(8):
                      //#######################
                          {

                            if(this->Turn_Right==true)
                                {path_=path_17_;this->state=17;Turn_Right=false;}
                            else
                                {path_=path_14_;this->state=14;}
                            break;

                          }
                  //############################

                      case(9):
                    //  #######################
                             {

                               if(this->Turn_Right==true)
                                   {path_=path_17_;this->state=17;Turn_Right=false;}
                               else
                                   {path_=path_14_;this->state=14;}

                               break;

                             }
                    // ############################3
                      case(10):path_=path_13_;this->state=13;Turn_Left=false;Turn_Right=false;break;
                      case(11):path_=path_16_;this->state=16;Turn_Left=false;Turn_Right=false;break;
                      case(12):path_=path_15_;this->state=15;Turn_Left=false;Turn_Right=false;break;

                      case(13):
                          if(this->Turn_Left==true)
                              {path_=path_1_;this->state=1;Turn_Left=false;Turn_Right=false;}
                          else if(this->Turn_Right==true)
                              {path_=path_10_;this->state=10;Turn_Left=false;Turn_Right=false;}
                          else
                              {path_=path_7_;this->state=7;Turn_Left=false;Turn_Right=false;}
                          break;

                      case(14):
                                {

		                  if(this->Turn_Left==true)
		                      {path_=path_9_;this->state=9;Turn_Left=false;Turn_Right=false;}
		                  else if(this->Turn_Right==true)
		                      {path_=path_3_;this->state=3;Turn_Left=false;Turn_Right=false;}
		                  else
		                      {path_=path_5_;this->state=5;Turn_Left=false;Turn_Right=false;}
		                  break;
				}
                      case(15):
                                {
                                      if(this->Turn_Left==true)
                                          {path_=path_12_;this->state=12;Turn_Left=false;Turn_Right=false;}
                                      else if(this->Turn_Right==true)
                                          {path_=path_2_;this->state=2;Turn_Left=false;Turn_Right=false;}
                                      else
                                          {path_=path_6_;this->state=6;Turn_Left=false;Turn_Right=false;}
                                      break;
                                    }
                      case(16):
                                {
                                      if(this->Turn_Left==true)
                                          {path_=path_4_;this->state=4;Turn_Left=false;Turn_Right=false;}
                                      else if(this->Turn_Right==true)
                                          {path_=path_11_;this->state=11;Turn_Left=false;Turn_Right=false;}
                                      else
                                          {path_=path_8_;this->state=8;Turn_Left=false;Turn_Right=false;}
                                      break;
                                }

                                /*/###########################################################3
                                 case(17):

                                   reconfigureServer_.setParam("/lateral_controller/path_topic", "/path_pylon");
                                   reconfigureServer_.setParam("/longitudinal_controller/velocity", 0.1);

                                    */




                        }
           }

}





void PathFromBag::read_bag(nav_msgs::Path::Ptr path_,std::string file_name ){//read ros bag file according to state

  //std::string file_name="t"+std::string(s);
  const std::vector<tf::tfMessage::ConstPtr> msgs_tfs = fromBag<tf::tfMessage>(file_name, "/tf");


       // Try to find a transform between stargazer and map
       // Here, wait for three seconds and cancel if no transform was found by then.

      Eigen::Affine3d tf_stargazer_to_map_eigen, tf_vehicle_to_camera_eigen;
      try {
          ros::Time now = ros::Time::now();
          const geometry_msgs::TransformStamped tf_stargazer_to_map_ros{tfBuffer_.lookupTransform(
              interface_.frame_id_map, interface_.frame_id_stargazer, ros::Time(0), ros::Duration(3))};
          tf::transformMsgToEigen(tf_stargazer_to_map_ros.transform, tf_stargazer_to_map_eigen);
          const geometry_msgs::TransformStamped tf_vehicle_to_camera_ros{tfBuffer_.lookupTransform(
              interface_.frame_id_camera, interface_.vehicle_frame_id, ros::Time(0), ros::Duration(3))};
          tf::transformMsgToEigen(tf_vehicle_to_camera_ros.transform, tf_vehicle_to_camera_eigen);
      } catch (const tf2::TransformException& ex) {
          ROS_ERROR_STREAM(ex.what());
          ros::shutdown();
      }

      path_->header.frame_id = interface_.frame_id_map;
      path_->header.stamp = ros::Time::now();
      geometry_msgs::PoseStamped pose_ros;
      pose_ros.header = path_->header;
      size_t count{0};
      for (auto const& msg_tfs : msgs_tfs) {
          Eigen::Affine3d tf_camera_to_stargazer_eigen;
          for (auto const& tf : msg_tfs->transforms) {
              if (tf.header.frame_id == interface_.frame_id_stargazer && tf.child_frame_id == interface_.frame_id_camera)
                  tf::transformMsgToEigen(tf.transform, tf_camera_to_stargazer_eigen);
              else if (tf.header.frame_id == interface_.frame_id_camera && tf.child_frame_id == interface_.frame_id_stargazer) {
                  tf::transformMsgToEigen(tf.transform, tf_camera_to_stargazer_eigen);
                  tf_camera_to_stargazer_eigen = tf_camera_to_stargazer_eigen.inverse();
              } else
                  continue;
              const Eigen::Affine3d tf_vehicle_to_map_eigen{tf_stargazer_to_map_eigen * tf_camera_to_stargazer_eigen *
                                                            tf_vehicle_to_camera_eigen};
              //poses_.emplace(tf.header.stamp.toSec(), tf_vehicle_to_map_eigen);
              tf::poseEigenToMsg(tf_vehicle_to_map_eigen, pose_ros.pose);
              path_->poses.emplace_back(pose_ros);
              count++;
          }
      }
      //ROS_DEBUG_STREAM("Found " << count << " poses in rosbag '" << interface_.bag_file_name << "'.");



}

////////////////////////////////////////////////////////////////////////////






/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void PathFromBag::reconfigureRequest(const ReconfigureConfig& config, uint32_t level) {
    interface_.fromConfig(config);
  }



} // namespace path_from_bag_ros_tool

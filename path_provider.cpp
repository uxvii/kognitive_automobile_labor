#include "path_provider.hpp"
#include <eigen_conversions/eigen_msg.h>
#include <tf/tfMessage.h>
#include <utils_ros/ros_console.hpp>



namespace path_provider_ros_tool {

PathProvider::PathProvider(ros::NodeHandle nh_public, ros::NodeHandle nh_private)
        : params_{nh_private}, reconfigure_server_{nh_private}, tf_listener_{tf_buffer_} {

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
        &PathProvider::callback_pixel_to_3d,
        this
        );

    sub_pcl_         =  nh_public.subscribe(
        params_.topic_pcl,   //params_.topic_pcl
        params_.msg_queue_size,
        &PathProvider::callback_pcl,
        this
        );//, this, _1, _2);


    /**
         * Set up the timer running at a fixed rate, calling "callbackTimer"
    */
    timer_ = nh_private.createTimer(ros::Rate(params_.timer_rate), &PathProvider::callbackTimer, this);

    /**
         * Show info after everything is started
    */
    utils_ros::showNodeInfo();

}

  void PathProvider::callback_pixel_to_3d(const std_msgs::Int32MultiArray::ConstPtr& array){
    /**
     transform from
     */
    //   camera to map

         Eigen::Affine3d tf_camera_to_map_eigen;
         Eigen::Affine3d tf_point_to_camera_eigen; //point to cam in eigen form
         Eigen::Affine3d tf_point_to_map_eigen;  //point to map

         try {
           ros::Time now = ros::Time::now();
           const geometry_msgs::TransformStamped tf_camera_to_map_ros{tf_buffer_.lookupTransform(
               params_.frame_id_map, params_.frame_id_camera, ros::Time(0), ros::Duration(3))};
           tf::transformMsgToEigen(tf_camera_to_map_ros.transform, tf_camera_to_map_eigen);

         } catch (const tf2::TransformException& e) {
             ROS_WARN_STREAM(e.what());
             return;
         }




         //have to leer out all the data in the path_
         path_->poses.clear();
        //nav_msgs::Path::Ptr path_{new nav_msgs::Path};

         //points to camera
        path_->header.frame_id = params_.frame_id_map;
        path_->header.stamp = ros::Time::now();


        geometry_msgs::PoseStamped tf_point_to_camera_pose_ros; //point to camera positon in ros form

        geometry_msgs::PoseStamped tf_point_to_map_pose_ros;

        tf_point_to_map_pose_ros.header = path_->header;


         for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); it=it+2){
             int u=*it;
             int v=*(it+1);
             float x=cloud[v*cloud.width+u].x;
             float y=cloud[v*cloud.width+u].y;
             float z=cloud[v*cloud.width+u].z;
              // have to transform  xyz rotation is 0 0 0, to geometry::pose

              if(x<200 && y<200 && z<200)
                {
                        tf_point_to_camera_pose_ros.pose.position.x=x;
                        tf_point_to_camera_pose_ros.pose.position.y=y;
                        tf_point_to_camera_pose_ros.pose.position.z=z;
                        tf_point_to_camera_pose_ros.pose.orientation.x =0;
                        tf_point_to_camera_pose_ros.pose.orientation.y =0;
                        tf_point_to_camera_pose_ros.pose.orientation.z =0;
                        tf_point_to_camera_pose_ros.pose.orientation.w =0;

                        tf::poseMsgToEigen(tf_point_to_camera_pose_ros.pose, tf_point_to_camera_eigen);//ros pos  to eigen

                        tf_point_to_map_eigen=tf_camera_to_map_eigen*tf_point_to_camera_eigen;
                        tf::poseEigenToMsg(tf_point_to_map_eigen, tf_point_to_map_pose_ros.pose);


                       path_->poses.emplace_back(tf_point_to_map_pose_ros);

                }


                
              }

    }


void PathProvider::callbackTimer(const ros::TimerEvent& timer_event) {
    path_->header.stamp = timer_event.current_expected;
    publisher_path_.publish(path_);
}


void PathProvider::callback_pcl(const sensor_msgs::PointCloud2ConstPtr& pcd){
    pcl::fromROSMsg (*pcd, cloud);

}






}

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <boost/thread/thread.hpp>
#include <Eigen/Dense>

#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl_ros/transforms.h>
#include <pcl/common/time.h>



using namespace std;

ros::Publisher point_pub;
std::string filename;
std::string frame_id;
bool debug_enable;

pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud (new pcl::PointCloud<pcl::PointXYZ>);


void pcCallback(const geometry_msgs::PoseStampedConstPtr& msg){
    
    pcl::ScopeTime t("pcdCallback");

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    
    geometry_msgs::Quaternion q = msg->pose.orientation;
    Eigen::Quaternionf quat(q.w, q.x, q.y, q.z);
    Eigen::Matrix3f rotation = quat.toRotationMatrix();
    Eigen::Vector3f translation(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    transform.block<3,3>(0,0) = rotation;
    transform.block<3,1>(0,3) = translation;
    if(debug_enable) {
      std::cout << "Printing transformation matrix... " << std::endl;
      std::cout << transform << std::endl;
      std::cout << "-------------" << std::endl;
      std::cout << translation.transpose() << std::endl;
      std::cout << "-------------" << std::endl;
      std::cout << rotation << std::endl;
      std::cout << "-------------\n\n" << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr model_tf (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud (*model_cloud, *model_tf, transform);

    model_tf->header.frame_id = frame_id;
    //model_tf->header.frame_id = msg->header.frame_id;
    sensor_msgs::PointCloud2 pub_msg;
    pcl::toROSMsg(*model_tf,pub_msg);
    pub_msg.header.stamp = msg->header.stamp;
    point_pub.publish(pub_msg);

    return;
}


int main (int argc, char** argv){

    ros::init(argc, argv, "pct");
    ros::NodeHandle nh("~");

    nh.param("filename", filename, std::string("yellow_tool_point_cloud.pcd"));
    nh.param("pub_frame_id", frame_id, std::string("camera_depth_optical_frame"));
    nh.param("debugging", debug_enable, bool(true));
    
    std::cout << "model_file:\t" << filename << std::endl;
    pcl::PCDReader reader;
    reader.read(filename,*model_cloud);

    ros::Subscriber tf_sub = nh.subscribe("input_transform",1, pcCallback);
    point_pub = nh.advertise<sensor_msgs::PointCloud2>("output_cloud",1);

    ros::spin();

    return 0;
}

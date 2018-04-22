//pcd_snapshot_ariac.cpp
// example of saving a depth camera snapshot to a pcd file

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> 
#include <sensor_msgs/PointCloud.h>
#include <pcl_ros/point_cloud.h> //use these to convert between PCL and ROS datatypes
//#include <pcl/ros/conversions.h>
#include <pcl/conversions.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>
#include <sensor_msgs/point_cloud_conversion.h>
// #include <sensor_msgs/point_cloud_iterator.h>



using namespace std;

bool got_depth_cam_image = false; //snapshot indicator
pcl::PointCloud<pcl::PointXYZ>::Ptr depth_cam_cloud2_ptr(new pcl::PointCloud<pcl::PointXYZ>); 
sensor_msgs::PointCloud2 depth_cam_pointcloud2_msg;
//void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
void depthCamCB(const sensor_msgs::PointCloudConstPtr& cloud) {

    if (!got_depth_cam_image) { // once only, to keep the data stable
        ROS_INFO("got new depth camera image");
        sensor_msgs::convertPointCloudToPointCloud2(*cloud,depth_cam_pointcloud2_msg);
        ROS_INFO_STREAM("depthcam msg header: "<<depth_cam_pointcloud2_msg.header<<endl);
        //depth_cam_pointcloud2_msg.height=50;
        //depth_cam_pointcloud2_msg.width=50;
        ROS_INFO("depthcam height, width = %d, %d",depth_cam_pointcloud2_msg.height,depth_cam_pointcloud2_msg.width);
        
        pcl::fromROSMsg(depth_cam_pointcloud2_msg, *depth_cam_cloud2_ptr);
        ROS_INFO("image has  %d * %d points", depth_cam_cloud2_ptr->width, depth_cam_cloud2_ptr->height);
        depth_cam_cloud2_ptr->width=50;
        depth_cam_cloud2_ptr->height=50;
        ROS_INFO("modified image has  %d * %d points", depth_cam_cloud2_ptr->width, depth_cam_cloud2_ptr->height);
        
        got_depth_cam_image = true;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "depthcam_snapshot_main"); //node name
    ros::NodeHandle nh;
    ros::Subscriber pointcloud_subscriber = nh.subscribe("/ariac/depth_camera_4", 1, depthCamCB);

    //spin until obtain a snapshot
    ROS_INFO("waiting for depthcam data");
    while (!got_depth_cam_image) {
        ROS_INFO("waiting...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("got snapshot; saving to file depthcam_snapshot.pcd");
    pcl::io::savePCDFile("depthcam_snapshot.pcd", *depth_cam_cloud2_ptr, true);

    return 0;
}

//box_inspector library header file
#ifndef BOX_INSPECTOR_H_
#define BOX_INSPECTOR_H_
#include <map>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Product.h>
#include <xform_utils/xform_utils.h>
using namespace std;

const int NUM_PART_TYPES=5;
//means to map part names to numerical codes, and vice-versa
//edit the following to add more parts;
//part ID's MUST start at 1 and MUST be sequential
std::map<std::string, int> mappings =
{
   {"gear_part",1},
   {"piston_rod_part",2},
   {"gasket_part",3},
   {"disk_part",4},
   {"pulley_part",5}
};

std::map<int, std::string> part_id_to_name_mappings =
{
   {1,"gear_part"},
   {2,"piston_rod_part"},
   {3,"gasket_part"},
   {4,"disk_part"},
   {5,"pulley_part"}
};

class BoxInspector
{
public:
   BoxInspector(ros::NodeHandle* nodehandle);

  void get_new_snapshot_from_box_cam();

  bool get_box_pose_wrt_world(geometry_msgs::PoseStamped &box_pose_wrt_world);
  

  void compute_shipment_poses_wrt_world(osrf_gear::Shipment shipment_wrt_box, 
          geometry_msgs::PoseStamped box_pose_wrt_world,
          vector<osrf_gear::Model>  &desired_models_wrt_world);
  

  void update_inspection(vector<osrf_gear::Model> desired_models_wrt_world,
       vector<osrf_gear::Model> &satisfied_models_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_actual_coords_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_desired_coords_wrt_world,
       vector<osrf_gear::Model> &missing_models_wrt_world,
       vector<osrf_gear::Model> &orphan_models_wrt_world);

  //operates on an image, computes model poses w/rt box, puts result in shipment_status
  bool model_poses_wrt_box(osrf_gear::LogicalCameraImage box_inspector_image, 
    osrf_gear::Shipment &shipment_status);
  //this version operates on member var box_inspector_image_
  bool model_poses_wrt_box(osrf_gear::Shipment &shipment_status);


private:
    std::map<std::string, int> part_id_mappings_;
    ros::NodeHandle nh_; 

    XformUtils xformUtils_;

    void box_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);   

    geometry_msgs::PoseStamped compute_stPose(geometry_msgs::Pose cam_pose,geometry_msgs::Pose part_pose);

    ros::Subscriber box_camera_subscriber_;
    osrf_gear::LogicalCameraImage box_inspector_image_;
    bool got_new_snapshot_;
};
#endif

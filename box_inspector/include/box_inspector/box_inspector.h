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
#include <bin_inventory/bin_inventory.h>
#include <inventory_msgs/Part.h>
#include <std_srvs/Trigger.h>
#include <order_manager/order_manager.h>
#include <inventory_msgs/Inventory.h>
#include <inventory_msgs/Part.h>
#include <osrf_gear/Product.h>
#include <osrf_gear/ConveyorBeltControl.h>
#include <sensor_msgs/LaserScan.h>
#include <osrf_gear/DroneControl.h>
#include <robot_behavior_interface/RobotBehaviorInterface.h>
//#include <kuka_move_as/KukaBehaviorActionServer.h>
#include<bin_inventory/bin_inventory.h>
#include<box_inspector/box_inspector.h>
using namespace std;


const double QUALITY_INSPECTION_MAX_WAIT_TIME = 2.0;
/*const int NUM_PART_TYPES=5;
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
*/
class BoxInspector
{
public:
   BoxInspector(ros::NodeHandle* nodehandle);

  bool get_new_snapshot_from_box_cam();

  bool get_box_pose_wrt_world(geometry_msgs::PoseStamped &box_pose_wrt_world);
  bool find_missing_parts(vector<osrf_gear::Model> desired_models_wrt_world, vector<osrf_gear::Model> &missing_wrt_world) ;
  bool find_orphan_parts(vector<osrf_gear::Model> desired_models_wrt_world, vector<osrf_gear::Model> &orphan_parts);
  bool get_bad_part_Q1(inventory_msgs::Part &bad_part);
  bool get_bad_part_Q2(inventory_msgs::Part &bad_part);
  bool find_faulty_part_Q1(const osrf_gear::LogicalCameraImage qual_sensor_image,inventory_msgs::Part &bad_part);
  bool find_faulty_part_Q2(const osrf_gear::LogicalCameraImage qual_sensor_image,inventory_msgs::Part &bad_part);
  bool get_observed_part_pose(inventory_msgs::Part place_part,inventory_msgs::Part &observed_part);
  bool get_grasped_part_pose_wrt_world(inventory_msgs::Part &observed_part);

    
  void model_to_part(osrf_gear::Model model, inventory_msgs::Part &part, unsigned short int location=inventory_msgs::Part::QUALITY_SENSOR_1);
  void compute_shipment_poses_wrt_world(osrf_gear::Shipment shipment_wrt_box, 
          geometry_msgs::PoseStamped box_pose_wrt_world,
          vector<osrf_gear::Model>  &desired_models_wrt_world);
  
  bool get_filtered_snapshots_from_box_cam(osrf_gear::LogicalCameraImage &filtered_box_camera_image);
  bool get_grasped_part_pose_wrt_world(geometry_msgs::PoseStamped &grasped_part_pose_wrt_world);


  bool update_inspection(vector<osrf_gear::Model> desired_models_wrt_world,
       vector<osrf_gear::Model> &satisfied_models_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_actual_coords_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_desired_coords_wrt_world,
       vector<osrf_gear::Model> &missing_models_wrt_world,
       vector<osrf_gear::Model> &orphan_models_wrt_world);


  bool update_inspection(vector<osrf_gear::Model> desired_models_wrt_world,
       vector<osrf_gear::Model> &satisfied_models_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_actual_coords_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_desired_coords_wrt_world,
       vector<osrf_gear::Model> &missing_models_wrt_world,
       vector<osrf_gear::Model> &orphan_models_wrt_world,
       vector<int> &part_indices_missing,
        vector<int> &part_indices_misplaced,
        vector<int> &part_indices_precisely_placed);

  //operates on an image, computes model poses w/rt box, puts result in shipment_status
  bool model_poses_wrt_box(osrf_gear::LogicalCameraImage box_inspector_image, 
    osrf_gear::Shipment &shipment_status);
  //this version operates on member var box_inspector_image_
  bool model_poses_wrt_box(osrf_gear::Shipment &shipment_status);
  bool compare_pose(geometry_msgs::Pose , geometry_msgs::Pose);
  bool compare_pose(geometry_msgs::PoseStamped, geometry_msgs::PoseStamped);
  bool compare_pose_approx(geometry_msgs::Pose pose_A, geometry_msgs::Pose pose_B);
  bool compare_pose_approx(geometry_msgs::PoseStamped, geometry_msgs::PoseStamped);

  bool post_dropoff_check(vector<osrf_gear::Model> desired_models_wrt_world,vector<osrf_gear::Model> &misplaced_models_desired_coords, vector<osrf_gear::Model> &misplaced_models_actual_coords);
  bool pre_dropoff_check(inventory_msgs::Part part, osrf_gear::Model model_actual, osrf_gear::Model model_desired);
  geometry_msgs::PoseStamped NOM_BOX1_POSE_WRT_WORLD,NOM_BOX2_POSE_WRT_WORLD;

private:
    std::map<std::string, int> part_id_mappings_;
    ros::NodeHandle nh_; 
    int BOX_INSPECTOR_TIMEOUT=2;
    XformUtils xformUtils_;

    void box_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);   
    osrf_gear::LogicalCameraImage qual_sensor_1_image_,qual_sensor_2_image_;

    void quality_sensor_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    ros::Subscriber quality_sensor_1_subscriber_;  
    void quality_sensor_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    ros::Subscriber quality_sensor_2_subscriber_;    

    geometry_msgs::PoseStamped compute_stPose(geometry_msgs::Pose cam_pose,geometry_msgs::Pose part_pose);
    bool qual_sensor_1_sees_faulty_part_,qual_sensor_2_sees_faulty_part_;
    ros::Subscriber box_camera_subscriber_;
    osrf_gear::LogicalCameraImage box_inspector_image_;
    bool got_new_snapshot_;
    inventory_msgs::Part bad_part_Qsensor1_,bad_part_Qsensor2_;
    bool got_new_Q1_image_;
    bool got_new_Q2_image_;
};
#endif

#ifndef SHIPMENT_FILLER_H_
#define SHIPMENT_FILLER_H_
#include <std_srvs/Trigger.h>
#include <order_manager/order_manager.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <ariac_xform_utils/ariac_xform_utils.h>
//#include <bin_inventory/bin_inventory.h>
#include <inventory_msgs/Inventory.h>
#include <inventory_msgs/Part.h>
#include <osrf_gear/Product.h>
//#include <osrf_gear/Order.h>
//#include <osrf_gear/Shipment.h>
#include <osrf_gear/ConveyorBeltControl.h>
#include <sensor_msgs/LaserScan.h>
#include <osrf_gear/DroneControl.h>
//#include <robot_move_as/RobotMove.h>
//#include <robot_move_as/RobotMoveActionServer.h> //old
#include <robot_behavior_interface/RobotBehaviorInterface.h>
#include <robot_behavior_interface/system_magic_numbers.h>
//#include <kuka_move_as/KukaBehaviorActionServer.h>
#include<bin_inventory/bin_inventory.h>
#include<box_inspector/box_inspector.h>

//const int BOX_CAM1_LOCATION_CODE = 1;
const int BOX_INSPECTION1_LOCATION_CODE = 2;
const int BOX_INSPECTION2_LOCATION_CODE = 3;
const int DRONE_DOCK_LOCATION_CODE = 4;



using namespace std;

class ShipmentFiller
{
public:
  ShipmentFiller(ros::NodeHandle* nodehandle);
  //bool fill_shipment(osrf_gear::Shipment shipment);
  bool remove_unwanted_parts(vector<osrf_gear::Model> desired_models_wrt_world);
  bool replace_faulty_parts_inspec1(osrf_gear::Shipment shipment);
  bool replace_faulty_parts_inspec2(osrf_gear::Shipment shipment);
  bool adjust_shipment_part_locations(vector<osrf_gear::Model> desired_models_wrt_world);
  bool adjust_part_location_before_dropoff(inventory_msgs::Part part);
  bool correct_dropped_part(osrf_gear::Shipment shipment);
  bool report_shipment_to_drone();
  bool advance_shipment_on_conveyor(int location_code);
  void set_drone_shipment_name(osrf_gear::Shipment shipment);
  bool get_bad_part_Q1(inventory_msgs::Part &bad_part);
  bool get_bad_part_Q2(inventory_msgs::Part &bad_part);
  void update_inventory();
  bool choose_shipment(osrf_gear::Shipment &shipment);
  bool check_order_update(osrf_gear::Shipment &shipment);
  bool current_order_has_been_filled(); //delete order from its vector
  bool current_shipment_has_been_filled();
  void model_to_part(osrf_gear::Model model, inventory_msgs::Part &part, unsigned short int location=inventory_msgs::Part::QUALITY_SENSOR_1);
  bool get_part_and_place_in_box(inventory_msgs::Inventory &current_inventory, inventory_msgs::Part place_part);
  bool get_part_and_prepare_place_in_box(inventory_msgs::Inventory &current_inventory, inventory_msgs::Part place_part);

   geometry_msgs::Pose compute_pose_part_wrt_box(geometry_msgs::Pose part_pose_wrt_world, geometry_msgs::PoseStamped box_pose_wrt_world);
  void modelvec_to_shipment(string shipment_name, vector<osrf_gear::Model> vec_of_models, geometry_msgs::PoseStamped box_pose, osrf_gear::Shipment &shipment);


  bool get_part_and_hold(inventory_msgs::Inventory &current_inventory, inventory_msgs::Part place_part);

  //bool test_pose_close_to_near_box_edge(geometry_msgs::Pose pose_wrt_box);
  //unsigned short int get_box_placement_location_code(geometry_msgs::Pose pose_wrt_box);
  
  RobotBehaviorInterface robotBehaviorInterface;
  OrderManager orderManager;
  BinInventory binInventory;
  
  //  bool pick_part_fnc(inventory_msgs::Part part); 
  //bool place_part_no_release(inventory_msgs::Part part);
  geometry_msgs::PoseStamped NOM_BOX1_POSE_WRT_WORLD,NOM_BOX2_POSE_WRT_WORLD;

  //bool select_part_from_inventory(part_name, bin_num, bin_part_pose_stamped);
  bool select_part_from_inventory(std::string part_name,int &bin_num,geometry_msgs::PoseStamped &part_pose);
  bool find_box(osrf_gear::LogicalCameraImage cam_image,double &y_val, geometry_msgs::Pose &cam_pose, geometry_msgs::Pose &box_pose);
  
  BoxInspector boxInspector;
  bool find_faulty_part_Q1(const osrf_gear::LogicalCameraImage qual_sensor_image,inventory_msgs::Part &bad_part); 
  bool find_faulty_part_Q2(const osrf_gear::LogicalCameraImage qual_sensor_image,inventory_msgs::Part &bad_part); 
  XformUtils xformUtils;
  
private:
  ros::NodeHandle nh_;
  //instantiate a ShipmentInspector object

  ros::ServiceClient conveyor_client_,drone_client_;
  osrf_gear::ConveyorBeltControl conveyor_svc_msg_GO_;
  osrf_gear::ConveyorBeltControl conveyor_svc_msg_STOP_;
  void box_camera_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
  ros::Subscriber box_camera_1_subscriber_;
  void box_camera_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
  ros::Subscriber box_camera_2_subscriber_; 
  ros::Subscriber drone_depot_laser_scan_subscriber_;
  void drone_depot_laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr & range_msg);
  osrf_gear::LogicalCameraImage box_cam_1_image_,box_cam_2_image_,box_cam_3_image_;
  osrf_gear::LogicalCameraImage qual_sensor_1_image_,qual_sensor_2_image_;
  void quality_sensor_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
  ros::Subscriber quality_sensor_1_subscriber_;  
  void quality_sensor_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
  ros::Subscriber quality_sensor_2_subscriber_;    
  // /ariac/quality_control_sensor_1   osrf_gear/LogicalCameraImage
  //bool find_box(osrf_gear::LogicalCameraImage cam_image,double &y_val);



  //geometry_msgs::PoseStamped box_cam_1_stamped_pose_;
  //geometry_msgs::PoseStamped box_cam_2_stamped_pose_;  
  
  inventory_msgs::Inventory inventory_msg_;
  geometry_msgs::PoseStamped box_1_stamped_pose_;
  geometry_msgs::PoseStamped box_2_stamped_pose_;  
  double box_cam_1_dist_to_go_;
  bool box_cam1_sees_box_; 
  double box_cam_2_dist_to_go_;
  bool box_cam2_sees_box_;   
  bool drone_depot_sensor_sees_box_;
  bool qual_sensor_1_sees_faulty_part_,qual_sensor_2_sees_faulty_part_;
  
  inventory_msgs::Part bad_part_Qsensor1_,bad_part_Qsensor2_;
  geometry_msgs::PoseStamped compute_stPose(geometry_msgs::Pose cam_pose,geometry_msgs::Pose part_pose);
  //geometry_msgs::PoseStamped compute_stPose(geometry_msgs::Pose pose_wrt_box,geometry_msgs::PoseStamped box_pose_wrt_world_);
  geometry_msgs::PoseStamped compute_stPose_part_in_box_wrt_world(geometry_msgs::Pose pose_wrt_box,geometry_msgs::PoseStamped box_pose_wrt_world);


  osrf_gear::DroneControl droneControl_; //set shipment_type

  bool got_new_Q1_image_;
  bool got_new_Q2_image_;
};
#endif

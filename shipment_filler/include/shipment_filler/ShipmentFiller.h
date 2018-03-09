#ifndef SHIPMENT_FILLER_H_
#define SHIPMENT_FILLER_H_
#include <order_manager/order_manager.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <ariac_xform_utils/ariac_xform_utils.h>
//#include <bin_inventory/bin_inventory.h>
#include <inventory_msgs/Inventory.h>
#include <inventory_msgs/Part.h>
//#include <osrf_gear/Order.h>
//#include <osrf_gear/Shipment.h>
#include <osrf_gear/ConveyorBeltControl.h>
#include <sensor_msgs/LaserScan.h>
#include <osrf_gear/DroneControl.h>
//#include <sensor_msgs/Range.h>

//const int BOX_CAM1_LOCATION_CODE = 1;
const int BOX_INSPECTION1_LOCATION_CODE = 2;
const int BOX_INSPECTION2_LOCATION_CODE = 3;
const int DRONE_DOCK_LOCATION_CODE = 4;

using namespace std;

class ShipmentFiller
{
public:
  ShipmentFiller(ros::NodeHandle* nodehandle);
  bool fill_shipment(osrf_gear::Shipment shipment);
  bool replace_faulty_parts_inspec1(osrf_gear::Shipment shipment);
  bool replace_faulty_parts_inspec2(osrf_gear::Shipment shipment);
  bool adjust_shipment_part_locations(osrf_gear::Shipment shipment);
  bool correct_dropped_part(osrf_gear::Shipment shipment);
  bool report_shipment_to_drone();
  bool advance_shipment_on_conveyor(int location_code);
  void set_drone_shipment_name(osrf_gear::Shipment shipment);
  bool get_bad_part_Q1(inventory_msgs::Part &bad_part);


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
  void box_camera_3_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
  ros::Subscriber box_camera_3_subscriber_;  
  ros::Subscriber drone_depot_laser_scan_subscriber_;
  void drone_depot_laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr & range_msg);
  osrf_gear::LogicalCameraImage box_cam_1_image_,box_cam_2_image_,box_cam_3_image_;
  osrf_gear::LogicalCameraImage qual_sensor_1_image_,qual_sensor_2_image_;
  void quality_sensor_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
  ros::Subscriber quality_sensor_1_subscriber_;  
  void quality_sensor_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
  ros::Subscriber quality_sensor_2_subscriber_;    
  // /ariac/quality_control_sensor_1   osrf_gear/LogicalCameraImage
  bool find_box(osrf_gear::LogicalCameraImage cam_image,double &y_val);
  double box_cam_1_dist_to_go_;
  bool box_cam1_sees_box_; 
  double box_cam_2_dist_to_go_;
  bool box_cam2_sees_box_;   
  double box_cam_3_dist_to_go_;
  bool box_cam3_sees_box_;   
  bool drone_depot_sensor_sees_box_;
  bool qual_sensor_1_sees_faulty_part_,qual_sensor_2_sees_faulty_part_;
  inventory_msgs::Part bad_part_Qsensor1_,bad_part_Qsensor2_;
  geometry_msgs::PoseStamped compute_stPose(geometry_msgs::Pose cam_pose,geometry_msgs::Pose part_pose);
  bool find_faulty_part_Q1(const osrf_gear::LogicalCameraImage qual_sensor_image,inventory_msgs::Part &bad_part); 
  bool find_faulty_part_Q2(const osrf_gear::LogicalCameraImage qual_sensor_image,inventory_msgs::Part &bad_part); 
  osrf_gear::DroneControl droneControl_; //set shipment_type
  XformUtils xformUtils;
  bool got_new_Q1_image_;
};
#endif

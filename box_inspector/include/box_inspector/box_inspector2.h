//box_inspector library header file
#ifndef BOX_INSPECTOR2_H_
#define BOX_INSPECTOR2_H_
#include <map>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/LaserScan.h>

#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Product.h>
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/DroneControl.h>

#include <xform_utils/xform_utils.h>
#include <bin_inventory/bin_inventory.h>
#include <inventory_msgs/Inventory.h>
#include <inventory_msgs/Part.h>

//#include <robot_behavior_interface/RobotBehaviorInterface.h>
//#include <order_manager/order_manager.h>

using namespace std;


const double QUALITY_INSPECTION_MAX_WAIT_TIME = 2.0;
const double ORIGIN_ERR_TOL = 0.009; // use competition requirements
const double ORIENTATION_ERR_TOL = 0.09; //ditto

const int CAM2 = 2;
const int CAM1 = 1;
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
class BoxInspector2
{
public:
   BoxInspector2(ros::NodeHandle* nodehandle);

  bool get_new_snapshot_from_box_cam(int cam_num=1); 
  bool get_new_snapshot_from_box_cam2();

  bool get_box_pose_wrt_world(geometry_msgs::PoseStamped &box_pose_wrt_world,int cam_num=1);
  bool get_box_pose_wrt_world2(geometry_msgs::PoseStamped &box_pose_wrt_world);
  
  bool find_missing_parts(vector<osrf_gear::Model> desired_models_wrt_world, vector<osrf_gear::Model> &missing_wrt_world,int cam_num=1) ;
  bool find_orphan_parts(vector<osrf_gear::Model> desired_models_wrt_world, vector<osrf_gear::Model> &orphan_parts,int cam_num=1);
  bool get_bad_part_Q(inventory_msgs::Part &bad_part,int cam_num=1);  
  bool get_bad_part_Q1(inventory_msgs::Part &bad_part);
  bool get_bad_part_Q2(inventory_msgs::Part &bad_part);
  
  bool find_faulty_part_Q(const osrf_gear::LogicalCameraImage qual_sensor_image,inventory_msgs::Part &bad_part, int cam_num=1);
  
  bool find_faulty_part_Q1(const osrf_gear::LogicalCameraImage qual_sensor_image,inventory_msgs::Part &bad_part);
  bool find_faulty_part_Q2(const osrf_gear::LogicalCameraImage qual_sensor_image,inventory_msgs::Part &bad_part);
  //bool get_observed_part_pose(inventory_msgs::Part place_part,inventory_msgs::Part &observed_part);
  bool get_grasped_part_pose_wrt_world(inventory_msgs::Part &observed_part, int cam_num=1);
  bool get_grasped_part_pose_wrt_world2(inventory_msgs::Part &observed_part);

    
  void model_to_part(osrf_gear::Model model, inventory_msgs::Part &part, unsigned short int location=inventory_msgs::Part::QUALITY_SENSOR_1);
  void compute_shipment_poses_wrt_world(osrf_gear::Shipment shipment_wrt_box, 
          geometry_msgs::PoseStamped box_pose_wrt_world,
          vector<osrf_gear::Model>  &desired_models_wrt_world);
  
  bool get_filtered_snapshots_from_box_cam(osrf_gear::LogicalCameraImage &filtered_box_camera_image,int cam_num=1);
  bool get_filtered_snapshots_from_box_cam2(osrf_gear::LogicalCameraImage &filtered_box_camera_image);

  //bool get_grasped_part_pose_wrt_world(geometry_msgs::PoseStamped &grasped_part_pose_wrt_world);
  //bool get_grasped_part_pose_wrt_world2(geometry_msgs::PoseStamped &grasped_part_pose_wrt_world);



/**
 * 
   * @param [in] desired_models_wrt_world a vector of models, specifying desired types and locations for 
   *    filling a box
   * @param satisfied_models_wrt_world  a vector of models found to match the packing list and are placed within tolerances
   * @param misplaced_models_actual_coords_wrt_world  vector of models from packing list that are present, but misplaced
   * @param misplaced_models_desired_coords_wrt_world vector of desired coordinates for the above, in the same order
   * @param missing_models_wrt_world  vector of models that are on the  packing list but not in the box
   * @param orphan_models_wrt_world  vector models that are in the box, but not on the packing list
 * @param [out] part_indices_missing
 * @param [out] part_indices_misplaced
 * @param [out] part_indices_precisely_placed
 * @return 
 */
  bool update_inspection(vector<osrf_gear::Model> desired_models_wrt_world,
       vector<osrf_gear::Model> &satisfied_models_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_actual_coords_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_desired_coords_wrt_world,
       vector<osrf_gear::Model> &missing_models_wrt_world,
       vector<osrf_gear::Model> &orphan_models_wrt_world,
       vector<int> &part_indices_missing,
        vector<int> &part_indices_misplaced,
        vector<int> &part_indices_precisely_placed,
        int cam_num=1);

  //operates on an image, computes model poses w/rt box, puts result in shipment_status
  bool model_poses_wrt_box(osrf_gear::LogicalCameraImage box_inspector_image, 
    osrf_gear::Shipment &shipment_status);
  //this version operates on member var box_inspector_image_
  bool model_poses_wrt_box(osrf_gear::Shipment &shipment_status);
  //test w/ ARIAC tolerances
  bool compare_pose(geometry_msgs::Pose , geometry_msgs::Pose);
  bool compare_pose(geometry_msgs::PoseStamped, geometry_msgs::PoseStamped);
  //test w/ larger tolerances
  bool compare_pose_approx(geometry_msgs::Pose pose_A, geometry_msgs::Pose pose_B);
  bool compare_pose_approx(geometry_msgs::PoseStamped, geometry_msgs::PoseStamped);

  bool post_dropoff_check(vector<osrf_gear::Model> desired_models_wrt_world,vector<osrf_gear::Model> &misplaced_models_desired_coords, 
    vector<osrf_gear::Model> &misplaced_models_actual_coords, int cam_num=1);
  bool pre_dropoff_check(inventory_msgs::Part part, osrf_gear::Model model_actual, osrf_gear::Model model_desired, int cam_num=1);
  geometry_msgs::PoseStamped NOM_BOX1_POSE_WRT_WORLD,NOM_BOX2_POSE_WRT_WORLD;

private:
    std::map<std::string, int> part_id_mappings_;
    ros::NodeHandle nh_; 
    int BOX_INSPECTOR_TIMEOUT=2;
    XformUtils xformUtils_;

    //only defined for boxcam 1; extend this
    void box_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);   
    void box_camera_callback2(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);   
    
    osrf_gear::LogicalCameraImage qual_sensor_1_image_,qual_sensor_2_image_;

    void quality_sensor_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    ros::Subscriber quality_sensor_1_subscriber_;  
    void quality_sensor_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    ros::Subscriber quality_sensor_2_subscriber_;    

    geometry_msgs::PoseStamped compute_stPose(geometry_msgs::Pose cam_pose,geometry_msgs::Pose part_pose);
    bool qual_sensor_1_sees_faulty_part_,qual_sensor_2_sees_faulty_part_;
    ros::Subscriber box_camera_subscriber_,box_camera_subscriber2_;
    osrf_gear::LogicalCameraImage box_inspector_image_,box_inspector_image2_;
    bool got_new_snapshot_,got_new_snapshot2_; //add equiv for boxcam 2
    inventory_msgs::Part bad_part_Qsensor1_,bad_part_Qsensor2_;
    bool got_new_Q1_image_;
    bool got_new_Q2_image_;
};
#endif

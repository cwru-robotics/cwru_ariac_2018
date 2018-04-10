//class BinInventory is constructed to do the following:
//execute binInventory.update() to force a refresh from all installed bin cameras;
//data from these cameras refresh the structure "inventory" from scratch

//access the inventory with functions including:
// num_parts(part_name): returns the number of parts, with part name as arg
// find_part(part_name,&bin,&pose): fills in the bin# and pose of first part in the list
// find_part(part_name,part_n,&bin,&pose): fills in bin# and pose of n'th part in the list

#ifndef BIN_INVENTORY_H_
#define BIN_INVENTORY_H_
#include <map>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <xform_utils/xform_utils.h> 
#include <inventory_msgs/Inventory.h>
#include <inventory_msgs/Part.h>

using namespace std;
const double BIN_INVENTORY_TIMEOUT=2.0; //max time to wait for an inventory update

//const int NUM_CAMERAS=5;  //MUST EDIT THIS VALUE TO ADD CAMERAS
const int NUM_CAMERAS=5;  //MUST EDIT THIS VALUE TO ADD CAMERAS 
//note:  ALSO must edit constructor to add cameras
//also must add more  subscribers and callback functions

const int NUM_PART_TYPES=5;
//edit the following to add more parts;
//part ID's MUST start at 1 and MUST be sequential
std::map<std::string, int> name_to_part_id_mappings =
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
//note: ALSO must edit initializePartMappings() add known part names and assign part IDs

/*
struct PartInventory {
  std::vector<geometry_msgs::PoseStamped> part_stamped_poses;
  std::vector<int> bins;
};
*/
class BinInventory
{
public:
 
  BinInventory(ros::NodeHandle* nodehandle);
  int num_parts(std::string); //return number of parts
  int num_parts(int part_id); //return number of parts, by part_id
  int num_parts(inventory_msgs::Inventory inventory, int part_id);
  int num_parts(inventory_msgs::Inventory inventory, std::string); //return number of parts

  //get bin number and pose of named part
  //bool find_part(std::string part_name,int &bin_num,geometry_msgs::PoseStamped &part_pose, int &partnum);
  //bool find_part(std::string part_name,int &bin_num,geometry_msgs::PoseStamped &part_pose) {
  //    int partnum;
  //   return find_part(part_name,bin_num,part_pose, partnum);
  //}
  //bool find_part(std::string part_name,inventory_msgs::Part &pick_part, int &partnum);
  //as above, but operate on provided inventory
  bool find_part(inventory_msgs::Inventory current_inventory,std::string part_name,inventory_msgs::Part &pick_part, int &partnum);
  

  bool update(); //updates entire inventory

  void print_inventory_msg();  
  void get_inventory(inventory_msgs::Inventory &inventory_msgs);
  void counts_all_part_types(std::vector<int> &parts_counts);
  bool remove_part_from_inventory(int part_id, int partnum);
   //this version operates on an inventory that is provided
  bool remove_part_from_inventory( int part_id, int partnum, inventory_msgs::Inventory &inventory);

private:
    std::map<std::string, int> part_id_mappings;
    ros::NodeHandle nh_; 

    XformUtils xformUtils;
   
    int num_part_types_; //how many part types are there?
    //std::map<std::string, int> mappings_;
    //these values used to assure forced refresh of camera data
    std::vector<bool> bin_camera_triggers_;
    
    //NEED TO UPDATE THIS IN CONSTRUCTOR TO ADD CAMERAS
    std::vector<int> camera_to_bin_mapping_;

    //allocate one subscriber for each logical camera viewing a bin
    //std::vector <ros::Subscriber> subscribersVec_;


    //store results of every camera's data
    std::vector<osrf_gear::LogicalCameraImage> logicalCamDataVec_;

    //EDIT THE FOLLOWING TO ADD CAMERA CALLBACK FNCS
    /// Called when a new LogicalCameraImage message is received.
    void logical_camera_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    void logical_camera_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    void logical_camera_3_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    void logical_camera_4_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    void logical_camera_5_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);    
    //EDIT THE FOLLOWING TO ADD CAMERA SUBSCRIBERS

    ros::Subscriber logical_camera_1_subscriber_;
    ros::Subscriber logical_camera_2_subscriber_;
    ros::Subscriber logical_camera_3_subscriber_;
    ros::Subscriber logical_camera_4_subscriber_;
    ros::Subscriber logical_camera_5_subscriber_;    
    void copy_logical_camera_data(const osrf_gear::LogicalCameraImage::ConstPtr image_msg,
       osrf_gear::LogicalCameraImage &image_data);

    //here is the main data storage: a vector of "PartInventory" objects
    //index this object by part_id, and can access each PartInventory object
    //std::vector<PartInventory> inventory_;
    inventory_msgs::Inventory inventory_msg_;
    void initializeSubscribers();
    void initializeInventory();
    void initializePartMappings();
    void update_camera_data(int cam_num,const osrf_gear::LogicalCameraImage::ConstPtr image_msg);
    void fillCamToBinMapping();
    bool all_cameras_updated();
    void clear_inventory();  
    void clear_inventory_msg();  

    geometry_msgs::PoseStamped compute_stPose(geometry_msgs::Pose cam_pose,geometry_msgs::Pose part_pose);


}; // note: a class definition requires a semicolon at the end of the definition
#endif

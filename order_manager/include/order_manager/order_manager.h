//order_filler class
#ifndef ORDER_MANAGER_H_
#define ORDER_MANAGER_H_
#include <string>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <xform_utils/xform_utils.h>
#include <bin_inventory/bin_inventory.h>
#include <inventory_msgs/Inventory.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Shipment.h>

using namespace std;

const int ORDER_VECTOR_PENDING = 0;
const int ORDER_VECTOR_PRIORITY= 1;
const int ORDER_VECTOR_UNFILLABLE=2;
const int ORDER_VECTOR_UPDATED = 3;
const string SHIPMENT_FILLED("shipment_filled");

class OrderManager
{
public:
  OrderManager(ros::NodeHandle* nodehandle);
  bool order_is_fillable(osrf_gear::Order order);
  void update_inventory();
  void print_parts_list(std::vector<int> parts_list);
  bool parts_available(std::vector<int> parts_list);
  bool choose_order(osrf_gear::Order &order);
  bool choose_shipment(osrf_gear::Shipment &shipment);
  //bool find_part(std::string part_name,int &bin_num,geometry_msgs::PoseStamped &part_pose);
  bool is_updated(osrf_gear::Order order);
  bool check_order_update(osrf_gear::Shipment &shipment) ;
  bool current_order_has_been_filled(); //delete order from its vector
  bool current_shipment_has_been_filled();
  void print_inventory(inventory_msgs::Inventory inventory_msg); 
  void print_inventory_succinct(inventory_msgs::Inventory inventory_msg);   
  void get_inventory(inventory_msgs::Inventory &inventory_msg); 

private:
  ros::NodeHandle nh_; 
  BinInventory * p_binInventory_;
  XformUtils xformUtils;
  std::vector<osrf_gear::Order> pending_orders_;
  std::vector<osrf_gear::Order> priority_orders_;
  std::vector<osrf_gear::Order> unfillable_orders_;
  std::vector<osrf_gear::Order> updated_orders_;
  osrf_gear::Order current_order_in_process_;
  int current_order_vector_code_;
  int current_order_index_;
  int current_shipment_index_;
  bool order_is_in_process_;
  bool shipment_is_in_process_;
  ros::Subscriber order_subscriber_;
  bool order_is_updated_;
  void order_callback(const osrf_gear::Order::ConstPtr & order_msg);
  geometry_msgs::PoseStamped target_pose_to_world_coords(geometry_msgs::Pose part_pose_wrt_box, 
     geometry_msgs::PoseStamped box_pose_wrt_world);
  bool is_priority(osrf_gear::Order order);
  void move_order_to_unfillable(int order_num,std::vector<osrf_gear::Order> order_vec);
  bool delete_from_order_queue(int order_index, std::vector<osrf_gear::Order> &order_vec);
  bool mark_shipment_filled(int order_index, int shipment_index, std::vector<osrf_gear::Order> &order_vec);  
  void test_order_complete(int current_order_index, int current_shipment_index, std::vector<osrf_gear::Order> &order_vec);
  void mark_shipments_unfilled(osrf_gear::Order &order);
  inventory_msgs::Inventory inventory_msg_;
  bool successfully_filled_order_;
//  int num_rcvd_new_orders_;

// {
//    ROS_INFO_STREAM("Received order:\n" << *order_msg);
//    received_orders_.push_back(*order_msg);
//  }


  
}; // note: a class definition requires a semicolon at the end of the definition

#endif


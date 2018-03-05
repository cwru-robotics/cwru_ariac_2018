//order_filler class
#ifndef ORDER_FILLER_H_
#define ORDER_FILLER_H_
#include <string>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ariac_xform_utils/ariac_xform_utils.h>
#include <bin_inventory/bin_inventory.h>
#include <osrf_gear/Order.h>
using namespace std;

class OrderFiller
{
public:
  OrderFiller(ros::NodeHandle* nodehandle);
  bool order_is_fillable(osrf_gear::Order order);

private:
  BinInventory binInventory;
  std::vector<osrf_gear::Order> pending_orders;
  std::vector<osrf_gear::Order> priority_orders;
  std::vector<osrf_gear::Order> unfillable_orders;
  ros::Subscriber order_subscriber_;
  void order_callback(const osrf_gear::Order::ConstPtr & order_msg);
//  int num_rcvd_new_orders_;

// {
//    ROS_INFO_STREAM("Received order:\n" << *order_msg);
//    received_orders_.push_back(*order_msg);
//  }


  
}; // note: a class definition requires a semicolon at the end of the definition

#endif


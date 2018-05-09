#include "ros/ros.h"
#include "optimizer_func/optimizer_func.h"

#include <sstream>

osrf_gear::Order current, priority;
ros::Time current_order_recvd, priority_order_recvd;

osrf_gear::Shipment testing_shipment;

char *shipment_names[] = {"no_active_shipment", "order_0_shipment_0", "order_0_shipment_1", "order_1_shipment_0", "order_1_shipment_1"};

// A quick and dirty simplification for creating test messages
#define ADD_PRODUCT(prod, type_string, xp, yp, zp, xo, yo, zo, wo)  {  \
    prod.type.append(type_string);					\
    prod.pose.position.x = xp;						\
    prod.pose.position.y = yp;						\
    prod.pose.position.z = zp;						\
    prod.pose.orientation.x = xo;					\
    prod.pose.orientation.y = yo;					\
    prod.pose.orientation.z = zo;					\
    prod.pose.orientation.w = wo;					\
  }

// Listening for the Orders from ARIAC
void orderCallback(const osrf_gear::Order::ConstPtr& msg) {

  // If this is order_0 or order_1, save it and take the time it was received.  If it is an update, save it, but the order time is not altered.
  if (! msg->order_id.compare("order_0")) {
    current_order_recvd.now();
    current = *msg;
  } else if (!msg->order_id.compare("order_0_update_0")) {
    current = *msg;
  } else if (!msg->order_id.compare("order_1")) {
    priority_order_recvd.now();
    priority = *msg;
  } else if (!msg->order_id.compare("order_1_update_0")) {
    priority = *msg;
  }
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "optimizer");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("optimizer", optimize_shipments);
  ROS_INFO("Ready to respond to optimization requests.");

  ros::Subscriber sub = n.subscribe("ariac/orders", 5, orderCallback);

  // ros::spin();
  
  // Below is testing stuff.

  current.shipments.resize(1);
  current.shipments[0].shipment_type.append("order_0_shipment_0");
  current.shipments[0].products.resize(6);

  ADD_PRODUCT(current.shipments[0].products[0], "gasket_part", 0.1, -0.2, 0.0, 0.0, 0.0, -sqrt(2)/2, sqrt(2)/2);
  ADD_PRODUCT(current.shipments[0].products[1], "gasket_part", -0.1, -0.2, 0.0, 0.0, 0.0, -sqrt(2)/2, sqrt(2)/2);
  ADD_PRODUCT(current.shipments[0].products[2], "piston_rod_part", 0.15, 0.15, 0.0, 0.0, 0.0, 0.0, 1.0);
  ADD_PRODUCT(current.shipments[0].products[3], "piston_rod_part", -0.15, 0.15, 0.0, 0.0, 0.0, 0.0, 1.0);
  ADD_PRODUCT(current.shipments[0].products[4], "piston_rod_part", 0.0, 0.15, 0.0, 0.0, 0.0, 1.0, 0.0);
  ADD_PRODUCT(current.shipments[0].products[5], "gear_part", 0.15, -0.1, 0.0, 0.0, 0.0, 0.0, 1.0);
  
  priority.shipments.resize(2);
  priority.shipments[0].shipment_type.append("order_1_shipment_0");
  priority.shipments[0].products.resize(6);
  priority.shipments[1].shipment_type.append("order_1_shipment_1");
  priority.shipments[1].products.resize(5);

  ADD_PRODUCT(priority.shipments[0].products[0], "pulley_part", -0.1, -0.2, 0.0, 1.0, 0.0, 0.0, 0.0);
  ADD_PRODUCT(priority.shipments[0].products[1], "piston_rod_part", 0.0, 0.15, 0.0, 0.0, 0.0, 1.0, 0.0);
  ADD_PRODUCT(priority.shipments[0].products[2], "gear_part", 0.15, 0.1, 0.0, 0.0, 0.0, 0.0, 1.0);
  ADD_PRODUCT(priority.shipments[0].products[3], "gear_part", 0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  ADD_PRODUCT(priority.shipments[0].products[4], "gear_part", 0.15, -0.2, 0.0, 0.0, 0.0, 0.0, 1.0);
  ADD_PRODUCT(priority.shipments[0].products[5], "gear_part", 0.15, -0.1, 0.0, 0.0, 0.0, 0.0, 1.0);
  
  ADD_PRODUCT(priority.shipments[1].products[0], "gasket_part", 0.1, -0.2, 0.0, 0.0, 0.0, -sqrt(2)/2, sqrt(2)/2);
  ADD_PRODUCT(priority.shipments[1].products[1], "gasket_part", -0.1, -0.2, 0.0, 0.0, 0.0, -sqrt(2)/2, sqrt(2)/2);
  ADD_PRODUCT(priority.shipments[1].products[2], "piston_rod_part", 0.15, 0.15, 0.0, 0.0, 0.0, 0.0, 1.0);
  ADD_PRODUCT(priority.shipments[1].products[3], "piston_rod_part", -0.15, 0.15, 0.0, 0.0, 0.0, 0.0, 1.0);
  ADD_PRODUCT(priority.shipments[1].products[4], "piston_rod_part", 0.0, 0.15, 0.0, 0.0, 0.0, 1.0, 0.0);

  osrf_gear::Shipment loaded_parts;
  loaded_parts.products.resize(3);

  loaded_parts.products[0] = current.shipments[0].products[1];
  loaded_parts.products[1] = current.shipments[0].products[2];
  loaded_parts.products[2] = current.shipments[0].products[5];
  
  // decision_func(loaded_parts, ros::Duration(0.0));

  optimizer_func::optimizer_msgs::Request req;
  optimizer_func::optimizer_msgs::Response res;
  osrf_gear::Shipment empty_shipment;
  
  req.loaded = loaded_parts;
  req.orphaned = empty_shipment;
  req.reposition = empty_shipment;
  req.missing = empty_shipment;

  current_order_recvd = ros::Time::now() - ros::Duration(135.0);
  priority_order_recvd = ros::Time::now();
  
  req.giving_up = optimizer_func::optimizer_msgs::Request::NOT_GIVING_UP;

  // Run the optimization routine
  // optimize_shipments(req, res);

  // Act like a proper service
  ros::spin();
  
  return 0;
}

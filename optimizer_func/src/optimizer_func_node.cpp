#include "ros/ros.h"
#include "optimizer_func/optimizer_func.h"

#include <sstream>


osrf_gear::Order current;
osrf_gear::Order priority;

ros::Time current_order_recvd;
ros::Time priority_order_recvd;

void orderCallback(const osrf_gear::Order::ConstPtr& msg) {

  if (! msg->order_id.compare("order_0")) {
    current_order_recvd.now();
    current = *msg;
  } else if (!msg->order_id.compare("order_0_update_0")) {
    current = *msg;
  } else if (!msg->order_id.compare("order_1")) {
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
  ros::init(argc, argv, "optimizer_tester");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("ariac/orders", 5, orderCallback);

  // Below is testing stuff.

  osrf_gear::Order current;
  current.shipments.resize(1);
  current.shipments[0].products.resize(6);

  osrf_gear::Order priority;
  priority.shipments.resize(2);
  priority.shipments[0].products.resize(6);

  osrf_gear::Shipment loaded_parts;
  loaded_parts.products.resize(3);
  
  current.shipments[0].products[0].type.append("gasket_part");
  current.shipments[0].products[0].pose.position.x = 0.1;
  current.shipments[0].products[0].pose.position.y = -0.2;
  current.shipments[0].products[0].pose.position.z = 0;
  current.shipments[0].products[0].pose.orientation.x = 0;
  current.shipments[0].products[0].pose.orientation.y = 0;
  current.shipments[0].products[0].pose.orientation.z = sqrt(2)/2;
  current.shipments[0].products[0].pose.orientation.w = sqrt(2)/2;

  current.shipments[0].products[1].type.append("gasket_part");
  current.shipments[0].products[1].pose.position.x = -0.1;
  current.shipments[0].products[1].pose.position.y = -0.2;
  current.shipments[0].products[1].pose.position.z = 0;
  current.shipments[0].products[1].pose.orientation.x = 0;
  current.shipments[0].products[1].pose.orientation.y = 0;
  current.shipments[0].products[1].pose.orientation.z = -sqrt(2)/2;
  current.shipments[0].products[1].pose.orientation.w = sqrt(2)/2;

  current.shipments[0].products[2].type.append("piston_rod_part");
  current.shipments[0].products[2].pose.position.x = 0.15;
  current.shipments[0].products[2].pose.position.y = 0.15;
  current.shipments[0].products[2].pose.position.z = 0;
  current.shipments[0].products[2].pose.orientation.x = 0;
  current.shipments[0].products[2].pose.orientation.y = 0;
  current.shipments[0].products[2].pose.orientation.z = 0;
  current.shipments[0].products[2].pose.orientation.w = 1;

  current.shipments[0].products[3].type = "piston_rod_part";
  current.shipments[0].products[3].pose.position.x = -0.15;
  current.shipments[0].products[3].pose.position.y = 0.15;
  current.shipments[0].products[3].pose.position.z = 0;
  current.shipments[0].products[3].pose.orientation.x = 0;
  current.shipments[0].products[3].pose.orientation.y = 0;
  current.shipments[0].products[3].pose.orientation.z = 0;
  current.shipments[0].products[3].pose.orientation.w = 1;

  current.shipments[0].products[4].type = "piston_rod_part";
  current.shipments[0].products[4].pose.position.x = 0;
  current.shipments[0].products[4].pose.position.y = 0.15;
  current.shipments[0].products[4].pose.position.z = 0;
  current.shipments[0].products[4].pose.orientation.x = 0;
  current.shipments[0].products[4].pose.orientation.y = 0;
  current.shipments[0].products[4].pose.orientation.z = 1;
  current.shipments[0].products[4].pose.orientation.w = 0;

  current.shipments[0].products[5].type = "gear_part";
  current.shipments[0].products[5].pose.position.x = 0.15;
  current.shipments[0].products[5].pose.position.y = -0.1;
  current.shipments[0].products[5].pose.position.z = 0;
  current.shipments[0].products[5].pose.orientation.x = 0;
  current.shipments[0].products[5].pose.orientation.y = 0;
  current.shipments[0].products[5].pose.orientation.z = 0;
  current.shipments[0].products[5].pose.orientation.w = 1;

  priority.shipments[0].products[0].type = "pulley_part";
  priority.shipments[0].products[0].pose.position.x = -0.1;
  priority.shipments[0].products[0].pose.position.y = -0.2;
  priority.shipments[0].products[0].pose.position.z = 0;
  priority.shipments[0].products[0].pose.orientation.x = 1;
  priority.shipments[0].products[0].pose.orientation.y = 0;
  priority.shipments[0].products[0].pose.orientation.z = 0;
  priority.shipments[0].products[0].pose.orientation.w = 0;

  priority.shipments[0].products[1].type = "piston_rod_part";
  priority.shipments[0].products[1].pose.position.x = 0;
  priority.shipments[0].products[1].pose.position.y = 0.15;
  priority.shipments[0].products[1].pose.position.z = 0;
  priority.shipments[0].products[1].pose.orientation.x = 0;
  priority.shipments[0].products[1].pose.orientation.y = 0;
  priority.shipments[0].products[1].pose.orientation.z = 1;
  priority.shipments[0].products[1].pose.orientation.w = 0;

  priority.shipments[0].products[2].type = "gear_part";
  priority.shipments[0].products[2].pose.position.x = 0.15;
  priority.shipments[0].products[2].pose.position.y = 0.1;
  priority.shipments[0].products[2].pose.position.z = 0;
  priority.shipments[0].products[2].pose.orientation.x = 0;
  priority.shipments[0].products[2].pose.orientation.y = 0;
  priority.shipments[0].products[2].pose.orientation.z = 0;
  priority.shipments[0].products[2].pose.orientation.w = 1;

  priority.shipments[0].products[3].type = "gear_part";
  priority.shipments[0].products[3].pose.position.x = 0.15;
  priority.shipments[0].products[3].pose.position.y = 0.0;
  priority.shipments[0].products[3].pose.position.z = 0;
  priority.shipments[0].products[3].pose.orientation.x = 0;
  priority.shipments[0].products[3].pose.orientation.y = 0;
  priority.shipments[0].products[3].pose.orientation.z = 0;
  priority.shipments[0].products[3].pose.orientation.w = 1;

  priority.shipments[0].products[4].type = "gear_part";
  priority.shipments[0].products[4].pose.position.x = 0.15;
  priority.shipments[0].products[4].pose.position.y = -0.2;
  priority.shipments[0].products[4].pose.position.z = 0;
  priority.shipments[0].products[4].pose.orientation.x = 0;
  priority.shipments[0].products[4].pose.orientation.y = 0;
  priority.shipments[0].products[4].pose.orientation.z = 0;
  priority.shipments[0].products[4].pose.orientation.w = 1;

  priority.shipments[0].products[5].type = "gear_part";
  priority.shipments[0].products[5].pose.position.x = 0.15;
  priority.shipments[0].products[5].pose.position.y = -0.1;
  priority.shipments[0].products[5].pose.position.z = 0;
  priority.shipments[0].products[5].pose.orientation.x = 0;
  priority.shipments[0].products[5].pose.orientation.y = 0;
  priority.shipments[0].products[5].pose.orientation.z = 0;
  priority.shipments[0].products[5].pose.orientation.w = 1;

  loaded_parts.products[0] = current.shipments[0].products[1];
  loaded_parts.products[1] = current.shipments[0].products[2];
  loaded_parts.products[2] = current.shipments[0].products[5];
  
  optimizer_func(current, ros::Time::now() - ros::Duration(50), priority, ros::Time::now(), 0, loaded_parts, ros::Duration(0.0));

  return 0;
}

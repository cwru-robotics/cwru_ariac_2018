#include <vector>

#include <ros/ros.h>

#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>

int main(int argc, char ** argv) {
  // Last argument is the default name of the node.
  ros::init(argc, argv, "order_sender");

  ros::NodeHandle node;

  ros::Publisher order_publisher = node.advertise<osrf_gear::Order>("/ariac/orders", 1);

  geometry_msgs::Pose part_target_pose;
  part_target_pose.position.x = 0.1;
  part_target_pose.position.y = -0.1;
  part_target_pose.position.z = 0.0;

  part_target_pose.orientation.x = 0.0;
  part_target_pose.orientation.y = 0.0;
  part_target_pose.orientation.z = 0.0;
  part_target_pose.orientation.w = 1.0;

  osrf_gear::Product product;
  product.type = "piston_rod_part";
  product.pose = part_target_pose;
  
  osrf_gear::Shipment shipment,shipment2;
  shipment.shipment_type = "order_0_shipment_0";
  shipment.products.push_back(product);

  osrf_gear::Order order;
  order.order_id = "dummy";
  order.shipments.push_back(shipment);
  shipment2=shipment;
  shipment2.shipment_type = "";
  order.shipments.push_back(shipment2); //make this 2 shipments
  

  
  for (int i=0;i<2;i++) 
  {
        ROS_INFO("sending dummy order to /ariac/orders");
      order_publisher.publish(order);
      ros::Duration(1.0).sleep();
  }

ROS_INFO("sending real order to /ariac/orders");
  order.order_id = "real_order";
  shipment.shipment_type = "order_1_shipment_0";
  shipment.products[0].type = "gear_part";
   order.shipments.clear();
   order.shipments.push_back(shipment);
   shipment2=shipment;
   shipment2.shipment_type = "order_1_shipment_1";   
   order.shipments.push_back(shipment2);   
   order_publisher.publish(order);
      ros::Duration(2.0).sleep();
      
}
//rosmsg show osrf_gear/Order
//string order_id
//osrf_gear/Shipment[] shipments
//  string shipment_type
//  osrf_gear/Product[] products
//    string type
//    geometry_msgs/Pose pose
//      geometry_msgs/Point position
//        float64 x
//        float64 y
//        float64 z
//      geometry_msgs/Quaternion orientation
 //       float64 x
//        float64 y
//        float64 z
 //       float64 w


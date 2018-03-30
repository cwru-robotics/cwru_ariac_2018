#include <vector>

#include <ros/ros.h>

#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>

//order contains shipments
//shipment contains products
//product specifies name and desired location w/rt box


int main(int argc, char ** argv) {
  // Last argument is the default name of the node.
  ros::init(argc, argv, "order_sender");

  ros::NodeHandle node;

  ros::Publisher order_publisher = node.advertise<osrf_gear::Order>("/ariac/orders", 1);
  osrf_gear::Shipment shipment;
  shipment.shipment_type = "order_0_shipment_0";
  
  osrf_gear::Order order;  
  order.order_id = "test_order1";

  osrf_gear::Product product;  
  geometry_msgs::Pose part_target_pose;  


  //gasket_part:
  product.type = "gasket_part";  
  part_target_pose.position.x = 0.03;
  part_target_pose.position.y = 0.094;
  part_target_pose.position.z = 0.0;
  part_target_pose.orientation.x = 0.0;
  part_target_pose.orientation.y = 0.0;
  part_target_pose.orientation.z = 0.385;
  part_target_pose.orientation.w = 0.923;
  product.pose = part_target_pose;
  shipment.products.push_back(product);    
 

  product.type = "piston_rod_part";  
  part_target_pose.position.x = 0.170;
  part_target_pose.position.y = 0.017;
  part_target_pose.position.z = 0.0;
  part_target_pose.orientation.x = 0.0;
  part_target_pose.orientation.y = 0.0;
  part_target_pose.orientation.z = 0.15;
  part_target_pose.orientation.w = 0.988;
  product.pose = part_target_pose;
  shipment.products.push_back(product);  

    product.type = "piston_rod_part";  
  part_target_pose.position.x = -0.127;
  part_target_pose.position.y = -0.080;
  part_target_pose.position.z = 0.0;
  part_target_pose.orientation.x = 0.0;
  part_target_pose.orientation.y = 0.0;
  part_target_pose.orientation.z = 0.707;
  part_target_pose.orientation.w = 0.707;
  product.pose = part_target_pose;
  shipment.products.push_back(product);  
  
    product.type = "piston_rod_part";  
  part_target_pose.position.x = -0.127;
  part_target_pose.position.y = 0.080;
  part_target_pose.position.z = 0.0;
  part_target_pose.orientation.x = 0.0;
  part_target_pose.orientation.y = 0.0;
  part_target_pose.orientation.z = 0.343;
  part_target_pose.orientation.w = 0.939;
  product.pose = part_target_pose;
  shipment.products.push_back(product);    
  
  order.shipments.push_back(shipment);
  

  
  for (int i=0;i<2;i++) 
  {
      ROS_INFO("sending  order to /ariac/orders");
      order_publisher.publish(order);
      ros::Duration(1.0).sleep();
  }

      
}


//dummy logical camera
#include <vector>

#include <ros/ros.h>

#include <osrf_gear/LogicalCameraImage.h>

void fill_image_msg(osrf_gear::LogicalCameraImage &fake_camera_image) {
  //camera frame w/rt world:
/*
pose: 
  position: 
    x: 0.571
    y: 0.615
    z: 1.133
  orientation: 
    x: 0.707140837031
    y: -7.31194888344e-14
    z: -0.707072723701
    w: -7.31265325397e-14
*/
  fake_camera_image.pose.position.x = 0.571;
  fake_camera_image.pose.position.y = 0.615;
  fake_camera_image.pose.position.z = 1.133;

  fake_camera_image.pose.orientation.x = 0.70714;
  fake_camera_image.pose.orientation.y = 0;
  fake_camera_image.pose.orientation.z =  -0.707;
  fake_camera_image.pose.orientation.w = 0;

  osrf_gear::Model model;
  /*
    type: "shipping_box"
    pose: 
      position: 
        x: 0.547977751747
        y: -0.00471660522237
        z: 0.0120577458563
      orientation: 
        x: -0.703839442432
        y: -0.00128393349494
        z: 0.710357909454
        w: -0.000176827787184
   */
  model.type = "shipping_box";
  model.pose.position.x = 0.548;
  model.pose.position.y = -0.005;
  model.pose.position.z = 0.012;  

  model.pose.orientation.x = -0.704;
  model.pose.orientation.y = -0.00128;
  model.pose.orientation.z = 0.71036;
  model.pose.orientation.w = -0.00017;

  fake_camera_image.models.push_back(model);

  model.type = "gear_part";
  model.pose.position.x = 0.544;
  model.pose.position.y = -0.190;
  model.pose.position.z = 0.113;  

  model.pose.orientation.x = -0.695;
  model.pose.orientation.y = -0.123;
  model.pose.orientation.z = 0.699;
  model.pose.orientation.w = 0.1136;

  fake_camera_image.models.push_back(model);

  model.type = "gasket_part";
  model.pose.position.x = 0.5455;
  model.pose.position.y = -0.099;
  model.pose.position.z = 0.0212;  

  model.pose.orientation.x = -0.661;
  model.pose.orientation.y = 0.272;
  model.pose.orientation.z = 0.6445;
  model.pose.orientation.w = -0.2712;

  fake_camera_image.models.push_back(model);

  model.type = "gasket_part";
  model.pose.position.x = 0.5435;
  model.pose.position.y = 0.0866;
  model.pose.position.z = -0.030;  

  model.pose.orientation.x = -0.6566;
  model.pose.orientation.y = 0.2814;
  model.pose.orientation.z = 0.64849;
  model.pose.orientation.w = -0.2628;

  fake_camera_image.models.push_back(model);

  model.type = "piston_rod_part";
  model.pose.position.x = 0.545;
  model.pose.position.y = -0.022;
  model.pose.position.z = -0.158;  

  model.pose.orientation.x = -0.6998;
  model.pose.orientation.y = 0.1070;
  model.pose.orientation.z = 0.6983;
  model.pose.orientation.w = -0.1054;

  fake_camera_image.models.push_back(model);

  model.type = "piston_rod_part";
  model.pose.position.x = 0.5466;
  model.pose.position.y = 0.0746;
  model.pose.position.z = 0.1394;  

  model.pose.orientation.x = -0.6624;
  model.pose.orientation.y = 0.2453;
  model.pose.orientation.z = 0.6659;
  model.pose.orientation.w = -0.2396;

  fake_camera_image.models.push_back(model);

}

int main(int argc, char ** argv) {
  // Last argument is the default name of the node.
  ros::init(argc, argv, "order_sender");

  ros::NodeHandle node;

  ros::Publisher logical_image_publisher = node.advertise<osrf_gear::LogicalCameraImage>("/ariac/box_camera_1", 1);
  osrf_gear::LogicalCameraImage fake_camera_image;
  fill_image_msg(fake_camera_image);

  while(ros::ok()) {
    logical_image_publisher.publish(fake_camera_image);
    ros::Duration(1.0).sleep();
  }

}

/*
osrf_gear/Model[] models
  string type
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
*/

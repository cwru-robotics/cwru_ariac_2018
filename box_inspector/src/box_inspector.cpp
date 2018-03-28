//box_inspector.cpp implementation of class/library
#include <box_inspector/box_inspector.h>
//#include "box_inspector_fncs.cpp" //more code, outside this file
#include "box_inspector_fncs_soln.cpp" //more code, outside this file

BoxInspector::BoxInspector(ros::NodeHandle* nodehandle) : nh_(*nodehandle) { //constructor
  //set up camera subscriber:
   box_camera_subscriber_ = nh_.subscribe("/ariac/box_camera_1", 1, &BoxInspector::box_camera_callback, this);
   got_new_snapshot_=false; //trigger to get new snapshots

}

//to request a new snapshot, set need_new_snapshot_ = true, and make sure to give a ros::spinOnce()
void BoxInspector::box_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    if (!got_new_snapshot_) {
        box_inspector_image_ = *image_msg;  //copy the current message to a member data var, i.e. freeze the snapshot
        got_new_snapshot_ =  true;
        ROS_INFO_STREAM("received box-camera image of: "<<box_inspector_image_<<endl);
        int n_models = box_inspector_image_.models.size();
        ROS_INFO("%d models seen ",n_models);
    }
}

//method to request a new snapshot from logical camera; blocks until snapshot is ready,
// then result will be in box_inspector_image_
void BoxInspector::get_new_snapshot_from_box_cam() {
  got_new_snapshot_= false;
  ROS_INFO("waiting for snapshot from camera");
  while (!got_new_snapshot_) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("got new snapshot");
}


//here is  the main fnc; provide a list of models, expressed as desired parts w/ poses w/rt box;
//get a box-camera logical image and  parse it
//populate the vectors as follows:
// satisfied_models_wrt_world: vector of models that match shipment specs, including precision location in box
// misplaced_models_wrt_world: vector of models that belong in the shipment, but are imprecisely located in box
// missing_models_wrt_world:  vector of models that are requested in the shipment, but not yet present in the box
// orphan_models_wrt_world: vector of models that are seen in the box, but DO NOT belong in the box
  void BoxInspector::update_inspection(vector<osrf_gear::Model> desired_models_wrt_world,
       vector<osrf_gear::Model> &satisfied_models_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_actual_coords_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_desired_coords_wrt_world,
       vector<osrf_gear::Model> &missing_models_wrt_world,
       vector<osrf_gear::Model> &orphan_models_wrt_world) {
  //WRITE ME!!!
  ROS_WARN("NEED TO WRITE update_inspection() ");
  got_new_snapshot_=false;
  while(!got_new_snapshot_) {
     ros::spinOnce(); // refresh camera image
     ros::Duration(0.5).sleep();
     ROS_INFO("waiting for logical camera image");
  }
  ROS_INFO("got new image");
  int num_parts_seen =  box_inspector_image_.models.size();
  ROS_INFO("update_inspection: box camera saw %d objects",num_parts_seen);
  orphan_models_wrt_world.clear(); //this will be empty, unless something very odd happens
  satisfied_models_wrt_world.clear();  //shipment will be complete when this matches parts/poses specified in shipment
  misplaced_models_actual_coords_wrt_world.clear();
  misplaced_models_desired_coords_wrt_world.clear();
  missing_models_wrt_world.clear();

  //THIS  IS WRONG...but an example of how to get models from image and sort into category vectors
  for (int i=0;i<num_parts_seen;i++) {
     orphan_models_wrt_world.push_back(box_inspector_image_.models[i]);
  }
  

}




//rosmsg show osrf_gear/LogicalCameraImage: 
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

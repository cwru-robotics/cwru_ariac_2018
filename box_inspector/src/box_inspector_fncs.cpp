//helper  function:
//given a camera pose and a part-pose (or box-pose) w/rt camera, compute part pose w/rt world
//xform_utils library should help here
geometry_msgs::PoseStamped BoxInspector::compute_stPose(geometry_msgs::Pose cam_pose,geometry_msgs::Pose part_pose) {

  geometry_msgs::PoseStamped stPose_part_wrt_world;
  //compute part-pose w/rt world and return as a pose-stamped message object
  ROS_WARN("NEED TO WRITE compute_stPose()");
  return stPose_part_wrt_world;
}

//given an image from logical camera, express model poses w/rt box:
//given an image, compute all models w/rt box and return in a "Shipment" object
void BoxInspector::model_poses_wrt_box(box_inspector::LogicalCameraImage box_inspector_image, 
    box_inspector::Shipment &shipment_status) {

}



//helper  function:
//given a camera pose and a part-pose (or box-pose) w/rt camera, compute part pose w/rt world
//xform_utils library should help here
geometry_msgs::PoseStamped BoxInspector::compute_stPose(geometry_msgs::Pose cam_pose,geometry_msgs::Pose part_pose) {

  geometry_msgs::PoseStamped stPose_part_wrt_world;
  //compute part-pose w/rt world and return as a pose-stamped message object
  ROS_WARN("NEED TO WRITE compute_stPose()");
  return stPose_part_wrt_world;
}

//given a shipment description that specifies desired parts and  poses with respect to box,
//convert this to poses of parts w/rt world;
//robot needs to know current and desired part poses  w/rt world
void BoxInspector::compute_shipment_poses_wrt_world(box_inspector::Shipment shipment_wrt_box, 
          vector<box_inspector::Model>  &desired_models_wrt_world)  {

  ROS_WARN("WRITE THIS FNC! compute_shipment_poses_wrt_world()");
  //compute and fill in terms in desired_models_wrt_world
}

//intent of this function is, get a snapshot from the box-inspection camera;
//parse the image data to see if a shipping box is present
//if not, return false
//if seen, transform box pose to box pose w/rt world frame,
//    copy data to reference arg box_pose_wrt_world
//    and return "true"
bool BoxInspector::get_box_pose_wrt_world(geometry_msgs::PoseStamped &box_pose_wrt_world) {
  geometry_msgs::Pose cam_pose, box_pose; //cam_pose is w/rt world, but box_pose is w/rt camera

  //get a new snapshot of the box-inspection camera:
  get_new_snapshot_from_box_cam();

  //ROS_INFO("got box-inspection camera snapshot");
  //look for box in model list:
    int num_models = box_inspector_image_.models.size(); //how many models did the camera see?
    if (num_models == 0) return false;
    string box_name("shipping_box"); //does a model match this name?
    box_inspector::Model model;
    cam_pose = box_inspector_image_.pose;
    ROS_INFO("box cam sees %d models", num_models);
    for (int imodel = 0; imodel < num_models; imodel++) {
        model = box_inspector_image_.models[imodel];
        string model_name(model.type);
        if (model_name == box_name) {
            box_pose = model.pose;
            ROS_INFO_STREAM("get_box_pose_wrt_world(): found box at pose"<<box_pose);
            //FINISH ME!!
            ROS_WARN("USE THIS INFO TO COMPUTE BOX POSE WRT WORLD AND  POPULATE box_pose_wrt_world");
            box_pose_wrt_world = compute_stPose(cam_pose,box_pose);
            return true;
        }
    }
    //if reach here, did not find shipping_box
    ROS_WARN("get_box_pose_wrt_world(): shipping-box not seen!");
    return false;
}

//helper  function:
//given a camera pose and a part-pose (or box-pose) w/rt camera, compute part pose w/rt world
//xform_utils library should help here

geometry_msgs::PoseStamped BoxInspector::compute_stPose(geometry_msgs::Pose cam_pose, geometry_msgs::Pose part_pose) {

    geometry_msgs::PoseStamped stPose_part_wrt_world;
    //compute part-pose w/rt world and return as a pose-stamped message object
    Eigen::Affine3d cam_wrt_world, part_wrt_cam, part_wrt_world;
    cam_wrt_world = xformUtils_.transformPoseToEigenAffine3d(cam_pose);
    part_wrt_cam = xformUtils_.transformPoseToEigenAffine3d(part_pose);
    part_wrt_world = cam_wrt_world*part_wrt_cam;
    geometry_msgs::Pose pose_part_wrt_world = xformUtils_.transformEigenAffine3dToPose(part_wrt_world);
    geometry_msgs::PoseStamped part_pose_stamped;
    part_pose_stamped.header.stamp = ros::Time::now();
    part_pose_stamped.header.frame_id = "world";
    part_pose_stamped.pose = pose_part_wrt_world;
    return part_pose_stamped;
}


//given an image, compute all models w/rt box and return in a "Shipment" object

bool BoxInspector::model_poses_wrt_box(osrf_gear::Shipment &shipment_status) {
    ROS_INFO("model_poses_wrt_box()");
    geometry_msgs::Pose cam_pose, box_pose_wrt_cam, model_pose_wrt_cam, part_pose_wrt_box;
    geometry_msgs::PoseStamped box_pose_wrt_world, part_pose_wrt_world;
    Eigen::Affine3d affine_cam_wrt_world, affine_part_wrt_cam, affine_part_wrt_box,
            affine_box_pose_wrt_world, affine_part_wrt_world;

    get_new_snapshot_from_box_cam();
    bool found_box = false;
    int i_box = 0;
    int num_models;

    num_models = box_inspector_image_.models.size();
    //parse the  image and compute model poses w/rt box; ignore  the box itself
    if (num_models == 0) {
        ROS_WARN("model_poses_wrt_box(): image has zero models");
        return false;
    }
    ROS_INFO("box cam sees %d models", num_models);

    //look for the box:
    string box_name("shipping_box"); //does a model match this name?
    osrf_gear::Model model;
    cam_pose = box_inspector_image_.pose;

    for (int imodel = 0; imodel < num_models; imodel++) {
        model = box_inspector_image_.models[imodel];
        string model_name(model.type);
        if (model_name == box_name) {
            box_pose_wrt_cam = model.pose;
            ROS_INFO_STREAM("box pose w/rt camera: " << box_pose_wrt_cam << endl);
            found_box = true;
            i_box = imodel; //remember where box is in the list of models
            box_pose_wrt_world = compute_stPose(cam_pose, box_pose_wrt_cam);
            affine_box_pose_wrt_world = xformUtils_.transformPoseToEigenAffine3d(box_pose_wrt_world);
            break;
        }
    }
    if (!found_box) {
        ROS_WARN("model_poses_wrt_box(): did not find box in view");
        return false;
    }

    osrf_gear::Product product;
    //if here, image contains box, and box pose w/rt world is in affine_box_pose_wrt_world
    for (int imodel = 0; imodel < num_models; imodel++) {
        if (imodel != i_box) { //if here, have a model NOT the box
            model = box_inspector_image_.models[imodel];
            string model_name(model.type);
            ROS_INFO_STREAM("model: " << model << endl);
            model_pose_wrt_cam = model.pose;
            part_pose_wrt_world = compute_stPose(cam_pose, model_pose_wrt_cam);
            ROS_INFO_STREAM("part pose wrt world: " << part_pose_wrt_world << endl);
            affine_part_wrt_world = xformUtils_.transformPoseToEigenAffine3d(part_pose_wrt_world);
            //  (A_box/world).inverse() * A_part/world   =   A_world/box * A_part/world = A_part/box
            affine_part_wrt_box = affine_box_pose_wrt_world.inverse() * affine_part_wrt_world;
            part_pose_wrt_box = xformUtils_.transformEigenAffine3dToPose(affine_part_wrt_box);
            //put this into "shipment"  object:
            //string shipment_type
            //box_inspector/Product[] products
            //  string type
            //  geometry_msgs/Pose pose    
            product.type = model.type;
            product.pose = part_pose_wrt_box;
            shipment_status.products.push_back(product);
        }
    }
    ROS_INFO_STREAM("resulting part poses w/rt box: " << shipment_status << endl);

}



//given a shipment description that specifies desired parts and  poses with respect to box,
//convert this to poses of parts w/rt world;
//robot needs to know current and desired part poses  w/rt world

void BoxInspector::compute_shipment_poses_wrt_world(osrf_gear::Shipment shipment_wrt_box,
        vector<osrf_gear::Model> &desired_models_wrt_world) {

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
    osrf_gear::Model model;
    cam_pose = box_inspector_image_.pose;
    ROS_INFO("box cam sees %d models", num_models);
    for (int imodel = 0; imodel < num_models; imodel++) {
        model = box_inspector_image_.models[imodel];
        string model_name(model.type);
        if (model_name == box_name) {
            box_pose = model.pose;
            ROS_INFO_STREAM("get_box_pose_wrt_world(): found box at pose " << box_pose << endl);
            //ROS_WARN("USE THIS INFO TO COMPUTE BOX POSE WRT WORLD AND  POPULATE box_pose_wrt_world");
            box_pose_wrt_world = compute_stPose(cam_pose, box_pose);
            ROS_INFO_STREAM("box_pose_wrt_world: " << box_pose_wrt_world << endl);
            return true;
        }
    }
    //if reach here, did not find shipping_box
    ROS_WARN("get_box_pose_wrt_world(): shipping-box not seen!");
    return false;
}

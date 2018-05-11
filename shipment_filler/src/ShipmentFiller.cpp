//ShipmentFiller class implementation
#include<shipment_filler/ShipmentFiller.h>
//int ans;

ShipmentFiller::ShipmentFiller(ros::NodeHandle* nodehandle) : nh_(*nodehandle), robotBehaviorInterface(nodehandle),
orderManager(nodehandle), boxInspector(nodehandle), binInventory(nodehandle) {
    conveyor_client_ = nh_.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
    //rosservice call /ariac/drone "shipment_type: order_0_shipment_0"

    drone_client_ = nh_.serviceClient<osrf_gear::DroneControl>("/ariac/drone");
    conveyor_svc_msg_GO_.request.power = 100.0;
    conveyor_svc_msg_STOP_.request.power = 0.0;
    ROS_INFO("warming up conveyor service: ");
    for (int i = 0; i < 3; i++) {//client.call(srv)
        conveyor_client_.call(conveyor_svc_msg_STOP_);
        //ROS_INFO_STREAM("response: "<<conveyor_svc_msg_STOP_);
        ros::Duration(0.1).sleep();
    }
    
    //  geometry_msgs::PoseStamped NOM_BOX1_POSE_WRT_WORLD,NOM_BOX2_POSE_WRT_WORLD;
    // assign hard-coded nominal vals for boxes at Q1 and Q1:
    //0.55, 0.61, 0.588; rpy = 0,0,0
    NOM_BOX1_POSE_WRT_WORLD.header.frame_id = "world";
    NOM_BOX1_POSE_WRT_WORLD.pose.position.x = 0.55;
    NOM_BOX1_POSE_WRT_WORLD.pose.position.y = 0.61;
    NOM_BOX1_POSE_WRT_WORLD.pose.position.z = 0.588;
    NOM_BOX1_POSE_WRT_WORLD.pose.orientation.x=0.0;
    NOM_BOX1_POSE_WRT_WORLD.pose.orientation.y=0.0;
    NOM_BOX1_POSE_WRT_WORLD.pose.orientation.z=0.0;
    NOM_BOX1_POSE_WRT_WORLD.pose.orientation.w=1.0;
   
    NOM_BOX2_POSE_WRT_WORLD=NOM_BOX1_POSE_WRT_WORLD;
    NOM_BOX2_POSE_WRT_WORLD.pose.position.y = 0.266;

    //ariac/box_camera_1
    box_camera_1_subscriber_ = nh_.subscribe("/ariac/box_camera_1", 1,
            &ShipmentFiller::box_camera_1_callback, this);
    box_cam_1_dist_to_go_ = 100.0; //init s.t. box is not yet under camera
    box_cam1_sees_box_ = false;
    //ariac/box_camera_2
    box_camera_2_subscriber_ = nh_.subscribe("/ariac/box_camera_2", 1,
            &ShipmentFiller::box_camera_2_callback, this);
    box_cam_2_dist_to_go_ = 100.0; //init s.t. box is not yet under camera
    box_cam2_sees_box_ = false;
    //ariac/box_camera_3
    //box_camera_3_subscriber_ = nh_.subscribe("/ariac/box_camera_3", 1,
    //        &ShipmentFiller::box_camera_3_callback, this);   
    //box_cam_3_dist_to_go_= 100.0; //init s.t. box is not yet under camera
    //box_cam3_sees_box_ = false;    

    quality_sensor_1_subscriber_ = nh_.subscribe("/ariac/quality_control_sensor_1", 1,
            &ShipmentFiller::quality_sensor_1_callback, this);
    qual_sensor_1_sees_faulty_part_ = false;

    quality_sensor_2_subscriber_ = nh_.subscribe("/ariac/quality_control_sensor_2", 1,
            &ShipmentFiller::quality_sensor_2_callback, this);
    qual_sensor_2_sees_faulty_part_ = false;

    drone_depot_laser_scan_subscriber_ = nh_.subscribe("/ariac/laser_profiler_drone_depot", 1,
            &ShipmentFiller::drone_depot_laser_scan_callback, this);

    drone_depot_sensor_sees_box_ = false;
    //  void drone_depot_prox_sensor_callback(const sensor_msgs::Range::ConstPtr & range_msg);

    orderManager.update_inventory();
    //rostopic info /ariac/proximity_sensor_drone_depot
    //Type: sensor_msgs/Range
}

void ShipmentFiller::update_inventory() {
    orderManager.update_inventory();
}

bool ShipmentFiller::choose_shipment(osrf_gear::Shipment &shipment) {
    return (orderManager.choose_shipment(shipment));
}

bool ShipmentFiller::current_order_has_been_filled() { //delete order from its vector
    return (orderManager.current_order_has_been_filled());
}

bool ShipmentFiller::current_shipment_has_been_filled() { //delete order from its vector
    return (orderManager.current_shipment_has_been_filled());
}

//fnc to convert a vector of poses to a shipment message
void ShipmentFiller::modelvec_to_shipment(string shipment_name, vector<osrf_gear::Model> vec_of_models, geometry_msgs::PoseStamped box_pose, osrf_gear::Shipment &shipment) {
    /* string shipment_type
       osrf_gear/Product[] products
       string type
    */
    geometry_msgs::Pose part_pose_wrt_box;
    int nproducts = vec_of_models.size();
    shipment.products.resize(nproducts);
    shipment.shipment_type = shipment_name.c_str();
    for (int i=0;i<nproducts;i++) {
        shipment.products[i].type = vec_of_models[i].type;
        //could convert part pose to pose w/rt box, and install it:
        part_pose_wrt_box =   compute_pose_part_wrt_box((vec_of_models[i]).pose, box_pose);
        shipment.products[i].pose = part_pose_wrt_box;
    }
}

void ShipmentFiller::box_camera_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    box_cam_1_image_ = *image_msg;
    double box_y_val; //cam_y_val;
    //bool ShipmentFiller::find_box(osrf_gear::LogicalCameraImage cam_image,double &y_val, 
    //    geometry_msgs::Pose &cam_pose, geometry_msgs::Pose &box_pose)
    geometry_msgs::Pose cam_pose, box_pose;
    box_cam1_sees_box_ = find_box(box_cam_1_image_, box_y_val, cam_pose, box_pose);
    if (box_cam1_sees_box_) {
        //ROS_INFO("cam_y_val = %f",cam_y_val);
        box_cam_1_dist_to_go_ = -box_y_val;
        //compute the box pose w/rt world
        box_1_stamped_pose_ = compute_stPose(cam_pose, box_pose);
        //correct this for known height:
        box_1_stamped_pose_.pose.position.z = BOX_SURFACE_HT_WRT_WORLD;
        //ROS_INFO("box_cam_1_dist_to_go_ = %f",box_cam_1_dist_to_go_);
    }
}

void ShipmentFiller::box_camera_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    box_cam_2_image_ = *image_msg;
    double box_y_val; //cam_y_val;
    //ROS_INFO("box cam2 calling find_box");
    geometry_msgs::Pose cam_pose, box_pose;
    box_cam2_sees_box_ = find_box(box_cam_2_image_, box_y_val, cam_pose, box_pose);

    if (box_cam2_sees_box_) {
        //ROS_INFO("box_cam2_sees box at cam_y_val = %f", box_y_val);
        box_cam_2_dist_to_go_ = -box_y_val;
        //ROS_INFO("box_cam_2_dist_to_go_ = %f",box_cam_2_dist_to_go_);
        box_2_stamped_pose_ = compute_stPose(cam_pose, box_pose);
    }
}

//NOT IN USE:

/*
void ShipmentFiller::box_camera_3_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    box_cam_3_image_ = *image_msg;
    double box_y_val; //cam_y_val;
    box_cam3_sees_box_ = find_box(box_cam_3_image_, box_y_val);
    if (box_cam3_sees_box_) {
        //ROS_INFO("cam_y_val = %f",cam_y_val);
        box_cam_3_dist_to_go_ = -box_y_val;
        //ROS_INFO("box_cam_1_dist_to_go_ = %f",box_cam_1_dist_to_go_);
    }
}
 */

void ShipmentFiller::quality_sensor_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    qual_sensor_1_image_ = *image_msg;
    //ROS_INFO("got Qsensor1 msg...");
    qual_sensor_1_sees_faulty_part_ = find_faulty_part_Q1(qual_sensor_1_image_, bad_part_Qsensor1_);
    got_new_Q1_image_ = true;
}

void ShipmentFiller::quality_sensor_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    qual_sensor_2_image_ = *image_msg;
    qual_sensor_2_sees_faulty_part_ = find_faulty_part_Q2(qual_sensor_2_image_, bad_part_Qsensor2_);
    got_new_Q2_image_ =true;
}

bool ShipmentFiller::find_faulty_part_Q1(const osrf_gear::LogicalCameraImage qual_sensor_image,
        inventory_msgs::Part &bad_part) {
    int num_bad_parts = qual_sensor_image.models.size();
    if (num_bad_parts == 0) return false;
    //if here, find a bad part and populate bad_part w/ pose in world coords
    osrf_gear::Model model = qual_sensor_image.models[0];
    geometry_msgs::Pose cam_pose, part_pose;
    geometry_msgs::PoseStamped stPose_part_wrt_world;
    cam_pose = qual_sensor_image.pose;
    part_pose = model.pose;
    bad_part.name = model.type;
    stPose_part_wrt_world = compute_stPose(cam_pose, part_pose);
    bad_part.pose = stPose_part_wrt_world;
    bad_part.location = inventory_msgs::Part::QUALITY_SENSOR_1;
    return true;
}

bool ShipmentFiller::check_order_update(osrf_gear::Shipment &shipment) {
	if(!orderManager.check_order_update(shipment)) {
		ROS_INFO("chill, no update");
		return 0;
	}
	else {
		ROS_INFO("UPDATED ORDER!");
		return 1;	
	}
}

bool ShipmentFiller::remove_unwanted_parts(vector<osrf_gear::Model> desired_models_wrt_world) {
	vector<osrf_gear::Model> orphan_parts;
	if(boxInspector.find_orphan_parts(desired_models_wrt_world,orphan_parts)) {
		ROS_INFO("Found unwanted parts");
		for(int i=0;i<orphan_parts.size();i++) {
			inventory_msgs::Part discard_part;
			bool go_on=true;
			model_to_part(orphan_parts[i],discard_part);
			if(!robotBehaviorInterface.pick_part_from_box(discard_part)) {
				ROS_INFO("unable to pick it up, ignoring");
				go_on=false;
				return 0;
			}
			if(go_on) {
				if(!robotBehaviorInterface.discard_grasped_part(discard_part)) {
					ROS_INFO("unable to discard, how do i deal with that");
				}
			}
		}
	return 1; //BAD RETURN! EITHER FIND LOGIC OR RETURN VOID
}
return 0;
}

bool ShipmentFiller::get_bad_part_Q1(inventory_msgs::Part &bad_part) {
    got_new_Q1_image_ = false;
    double wait_time = 0;
    double dt = 0.1;
    while ((wait_time<QUALITY_INSPECTION_MAX_WAIT_TIME)&&!got_new_Q1_image_) {
        wait_time+=dt;
        ros::spinOnce();
        ros::Duration(dt).sleep();
    }
    if (wait_time>= QUALITY_INSPECTION_MAX_WAIT_TIME) {
        ROS_WARN("timed  out waiting for quality inspection cam1");
        return false;
    }
    //if here, then got an update from Q1 cam:
    bad_part = bad_part_Qsensor1_;
    return qual_sensor_1_sees_faulty_part_;
}

bool ShipmentFiller::get_bad_part_Q2(inventory_msgs::Part &bad_part) {
    got_new_Q2_image_ = false;
    double wait_time = 0;
    double dt = 0.1;
    while ((wait_time<QUALITY_INSPECTION_MAX_WAIT_TIME)&&!got_new_Q2_image_) {
        wait_time+=dt;
        ros::spinOnce();        
        ros::Duration(dt).sleep();
    }
    if (wait_time>= QUALITY_INSPECTION_MAX_WAIT_TIME) {
        ROS_WARN("timed  out waiting for quality inspection cam2");
        return false;
    }
    //if here, then got an update from Q2 cam:
    bad_part = bad_part_Qsensor2_;
    return qual_sensor_2_sees_faulty_part_;    
}

bool ShipmentFiller::find_faulty_part_Q2(const osrf_gear::LogicalCameraImage qual_sensor_image,
        inventory_msgs::Part &bad_part) {
    int num_bad_parts = qual_sensor_image.models.size();
    if (num_bad_parts == 0) return false;
    //if here, find a bad part and populate bad_part w/ pose in world coords
    osrf_gear::Model model = qual_sensor_image.models[0];
    geometry_msgs::Pose cam_pose, part_pose;
    geometry_msgs::PoseStamped stPose_part_wrt_world;
    cam_pose = qual_sensor_image.pose;
    part_pose = model.pose;
    bad_part.name = model.type;
    stPose_part_wrt_world = compute_stPose(cam_pose, part_pose);
    bad_part.pose = stPose_part_wrt_world;
    bad_part.location = inventory_msgs::Part::QUALITY_SENSOR_2;
    return true;
}

geometry_msgs::PoseStamped ShipmentFiller::compute_stPose(geometry_msgs::Pose cam_pose, geometry_msgs::Pose part_pose) {
    Eigen::Affine3d cam_wrt_world, part_wrt_cam, part_wrt_world;
    cam_wrt_world = xformUtils.transformPoseToEigenAffine3d(cam_pose);
    part_wrt_cam = xformUtils.transformPoseToEigenAffine3d(part_pose);
    part_wrt_world = cam_wrt_world*part_wrt_cam;
    geometry_msgs::Pose pose_part_wrt_world = xformUtils.transformEigenAffine3dToPose(part_wrt_world);
    geometry_msgs::PoseStamped part_pose_stamped;
    part_pose_stamped.header.stamp = ros::Time::now();
    part_pose_stamped.header.frame_id = "world";
    part_pose_stamped.pose = pose_part_wrt_world;
    return part_pose_stamped;
}


//convert from part_pose_wrt_world to part_pose_wrt_box
geometry_msgs::Pose ShipmentFiller::compute_pose_part_wrt_box(geometry_msgs::Pose part_pose_wrt_world, geometry_msgs::PoseStamped box_pose_wrt_world) {
    geometry_msgs::PoseStamped part_pose_stamped;
    Eigen::Affine3d affine_box_wrt_world, affine_part_wrt_box, affine_part_wrt_world;
    affine_box_wrt_world = xformUtils.transformPoseToEigenAffine3d(box_pose_wrt_world);
    affine_part_wrt_world = xformUtils.transformPoseToEigenAffine3d(part_pose_wrt_world);
    affine_part_wrt_box = affine_part_wrt_box*affine_box_wrt_world.inverse();
    geometry_msgs::Pose part_pose_wrt_box = xformUtils.transformEigenAffine3dToPose(affine_part_wrt_box);
    return part_pose_wrt_box;
}

void ShipmentFiller::drone_depot_laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr & scan_msg) {
    double max_range = scan_msg->range_max;
    //double min_sensed_range = max_range;
    //ROS_INFO("max_range = %f", max_range);
    drone_depot_sensor_sees_box_ = false;
    int num_rays = scan_msg->ranges.size();
    for (int iray = 0; iray < num_rays; iray++) {
        if (scan_msg->ranges[iray] < max_range - 0.01) {
            drone_depot_sensor_sees_box_ = true;
            //ROS_INFO("shipment seen at drone loading dock");
        }
    }
}

bool ShipmentFiller::find_box(osrf_gear::LogicalCameraImage cam_image, double &y_val,
        geometry_msgs::Pose &cam_pose, geometry_msgs::Pose &box_pose) {
    int num_models = cam_image.models.size();
    if (num_models == 0) return false;
    string box_name("shipping_box");
    //cam_y_val = cam_image.pose.position.y;
    osrf_gear::Model model;
    cam_pose = cam_image.pose;
    //ROS_INFO("box cam sees %d models", num_models);
    for (int imodel = 0; imodel < num_models; imodel++) {
        model = cam_image.models[imodel];
        string model_name(model.type);
        if (model_name == box_name) {
            y_val = model.pose.position.y;
            box_pose = model.pose;
            //ROS_INFO("found box at y_val = %f", y_val);
            return true;
        }
    }
    //if reach here, did not find shipping_box
    return false;
}

void part_to_model(inventory_msgs::Part part, osrf_gear::Model &model) {
    model.type = part.name;
    model.pose = part.pose.pose;

}

void ShipmentFiller::model_to_part(osrf_gear::Model model, inventory_msgs::Part &part, unsigned short int location) {
    part.name = model.type;
    part.pose.pose = model.pose;
    part.location = location; //by default
}

bool ShipmentFiller::get_part_and_place_in_box(inventory_msgs::Inventory &current_inventory, inventory_msgs::Part place_part) {
    //try to find part in inventory;
    //try to pick this part--and delete it from current inventory
    //try to place  part in box and release;
    //perform inspection/correction outside this func
    //a return of "true" does not confirm success; return of "false" is a known  failure
    bool go_on=true; 
    int ans;
    
    std::string part_name(place_part.name);
    ROS_INFO_STREAM("looking for part " << part_name << endl);
    int partnum_in_inventory;
    bool part_in_inventory = true; 
    inventory_msgs::Part pick_part;

    //while (part_in_inventory) {  //persistently rety pick/place until success or until out of inventory     
      //bool go_on=true; 
      part_in_inventory = binInventory.find_part(current_inventory, part_name, pick_part, partnum_in_inventory);
      if (!part_in_inventory) {
        ROS_WARN("could not find desired  part in inventory; giving up on process_part()");
        return false; //nothing more can be done     
      }
      ROS_INFO_STREAM("found part: " << pick_part << endl);
      ROS_INFO("attempting pick...");
        //manually remove this part from inventory, in case cameras are down:
        //  bool remove_part_from_inventory(int part_id, int partnum);
        int part_id = name_to_part_id_mappings[part_name];
        int nparts_in_inventory = binInventory.num_parts(current_inventory,part_id);
        ROS_INFO("number of parts of part_id= %d = %d",part_id,nparts_in_inventory);
        binInventory.remove_part_from_inventory(part_id, partnum_in_inventory,current_inventory); //don't  try this part again
        //int BinInventory::num_parts(inventory_msgs::Inventory inventory, int part_id) {
        nparts_in_inventory = binInventory.num_parts(current_inventory,part_id);
        ROS_INFO("after removal, %d parts left ",nparts_in_inventory);
        ROS_INFO("attempting to pick part");
        cout<<"enter 1:";
        if (!robotBehaviorInterface.pick_part_from_bin(pick_part)) {
            ROS_INFO("pick failed");
            go_on = false;
            return false;
            //gripperInterface_.release();     
        }
        if (go_on) {
            ROS_INFO("attempting to place part: ");
            cout<<"enter 1: ";
            ROS_INFO_STREAM("part to be placed: " << place_part << endl);
            if (!robotBehaviorInterface.place_part_in_box_no_release(place_part)) {
                ROS_INFO("placement failed");
                go_on=false;    
                return false;
            }
        }
        //loop back to retry with another part;
        //should do a box inspection, though,  to make sure part is not dropped  in box, occluding desired destination   
        if (go_on) {
            ROS_INFO("part placement (no release) was successful!");
            return true;
        }
    }
//}

bool ShipmentFiller::get_part_and_hold(inventory_msgs::Inventory &current_inventory, inventory_msgs::Part place_part) {
    //try to find part in inventory;
    //try to pick this part--and delete it from current inventory
    //try to place  part in box and release;
    //perform inspection/correction outside this func
    //a return of "true" does not confirm success; return of "false" is a known  failure
    bool go_on=true; 
    int ans;
    
    std::string part_name(place_part.name);
    ROS_INFO_STREAM("looking for part " << part_name << endl);
    int partnum_in_inventory;
    bool part_in_inventory = true; 
    inventory_msgs::Part pick_part;

    while (part_in_inventory) {  //persistently rety pick/place until success or until out of inventory     
      bool go_on=true; 
      part_in_inventory = binInventory.find_part(current_inventory, part_name, pick_part, partnum_in_inventory);
      if (!part_in_inventory) {
        ROS_WARN("could not find desired  part in inventory; giving up on process_part()");
        return false; //nothing more can be done     
      }
      ROS_INFO_STREAM("found part: " << pick_part << endl);
      ROS_INFO("attempting pick...");
        //manually remove this part from inventory, in case cameras are down:
        //  bool remove_part_from_inventory(int part_id, int partnum);
        int part_id = name_to_part_id_mappings[part_name];
        int nparts_in_inventory = binInventory.num_parts(current_inventory,part_id);
        ROS_INFO("number of parts of part_id= %d = %d",part_id,nparts_in_inventory);
        binInventory.remove_part_from_inventory(part_id, partnum_in_inventory,current_inventory); //don't  try this part again
        //int BinInventory::num_parts(inventory_msgs::Inventory inventory, int part_id) {
        nparts_in_inventory = binInventory.num_parts(current_inventory,part_id);
        ROS_INFO("after removal, %d parts left ",nparts_in_inventory);
        ROS_INFO("attempting to pick part");
        //cout<<"enter 1:";
        //cin>>ans;
        if (!robotBehaviorInterface.pick_part_from_bin(pick_part)) {
            ROS_INFO("pick failed");
            go_on = false;
            //return false; //REMOVE THIS IF NEEDED
            //gripperInterface_.release();     
        }
        if(go_on) {
            return true;
        }
    }
}


//for this version, stop at approach pose before part dropoff
//persist re-trying for up to MAX_TRIES attempts; return true or false for success/failure
bool ShipmentFiller::get_part_and_prepare_place_in_box(inventory_msgs::Inventory &current_inventory, inventory_msgs::Part place_part) {
    //try to find part in inventory;
    //try to pick this part--and delete it from current inventory
    //try to place  part in box and release;
    //perform inspection/correction outside this func
    //a return of "true" does not confirm success; return of "false" is a known  failure
    const int MAX_TRIES = 3;
    int num_tries =0;
    bool go_on=true; 
    int ans;
    
    std::string part_name(place_part.name);
    ROS_INFO_STREAM("looking for part " << part_name << endl);
    int partnum_in_inventory;
    bool part_in_inventory = true; 
    inventory_msgs::Part pick_part;

    while (part_in_inventory&& (num_tries<MAX_TRIES)) {  //persistently retry pick/place until success or until out of inventory     
      bool go_on=true; 
      num_tries++;
      //std::string part_name(place_part.name);
      part_in_inventory = binInventory.find_part(current_inventory, part_name, pick_part, partnum_in_inventory);
      if (!part_in_inventory) {
        ROS_WARN("could not find desired  part in inventory; giving up on process_part()");
        return false; //nothing more can be done     
      }
      ROS_INFO_STREAM("found part: " << pick_part << endl);
      ROS_INFO("attempting pick...");
        //manually remove this part from inventory, in case cameras are down:
        //  bool remove_part_from_inventory(int part_id, int partnum);
        int part_id = name_to_part_id_mappings[part_name];
        int nparts_in_inventory = binInventory.num_parts(current_inventory,part_id);
        ROS_INFO("number of parts of part_id= %d = %d",part_id,nparts_in_inventory);
        binInventory.remove_part_from_inventory(part_id, partnum_in_inventory,current_inventory); //don't  try this part again
        //int BinInventory::num_parts(inventory_msgs::Inventory inventory, int part_id) {
        nparts_in_inventory = binInventory.num_parts(current_inventory,part_id);
        ROS_INFO("after removal, %d parts left ",nparts_in_inventory);
        ROS_INFO("attempting to pick part");
        //cout<<"enter 1:";
        //cin>>ans;
        if (!robotBehaviorInterface.pick_part_from_bin(pick_part)) {
            ROS_INFO("pick failed");
            go_on = false;
            //return false; //REMOVE THIS IF NEEDED
            //gripperInterface_.release();     
        }

       

        if (go_on) {
            ROS_INFO("attempting to move part to pre-dropoff pose: ");
            //cout<<"enter 1: ";
            //cin>>ans;
            ROS_INFO_STREAM("part to be placed: " << place_part << endl);
            //    bool move_part_to_approach_pose(inventory_msgs::Part part,double timeout = MAX_ACTION_SERVER_WAIT_TIME); 

            if (!robotBehaviorInterface.move_part_to_approach_pose(place_part)) {
                ROS_INFO("could not move to approach pose");
                go_on=false; 
                robotBehaviorInterface.discard_grasped_part(place_part);
                //return false;  // REMOVE THIS IF NEEDED
            }
        }
        //loop back to retry with another part;
        //should do a box inspection, though,  to make sure part is not dropped  in box, occluding desired destination   
        if (go_on) {
            ROS_INFO("get_part_and_prepare_place_in_box was successful!");
            return true;
        }
    }
    return false;
}


    /*
    bool ShipmentFiller::fill_shipment(osrf_gear::Shipment shipment) {
        ROS_INFO("attempting to fill shipment");
        ROS_INFO_STREAM("shipment: " << shipment);
        inventory_msgs::Part pick_part, place_part;
        int num_parts = shipment.products.size();
        int bin_num;
        bool robot_move_ok;
        osrf_gear::Product product;
        geometry_msgs::PoseStamped bin_part_pose_stamped, box_part_pose_stamped;
        geometry_msgs::Pose pose_wrt_box;
        //ROS_INFO("moving to bin3 cruise pose");
        //robotMove.toPredefinedPose(robot_move_as::RobotMoveGoal::BIN3_CRUISE_POSE);

        ROS_INFO("updating inventory");
        orderManager.update_inventory();
        //void OrderManager::get_inventory(inventory_msgs::Inventory &inventory_msg)
        //orderManager.get_inventory(inventory_msg_);
    
        bool move_status;
        //bool destination_close_to_near_box_edge=false; //if too close, may need wrist flip
        //  unsigned short int get_box_placement_location_code(geometry_msgs::Pose pose_wrt_box);
        unsigned short int box_placement_location_code;
        for (int ipart = 0; ipart < num_parts; ipart++) {
            product = shipment.products[ipart];
            string part_name(product.type);
        
            ROS_INFO_STREAM("part " << ipart << " name is: " << part_name << endl);
            //bool ShipmentFiller::select_part_from_inventory(std::string part_name,int &bin_num,geometry_msgs::PoseStamped &part_pose) {

            if (!select_part_from_inventory(part_name, bin_num, bin_part_pose_stamped)) {
                ROS_WARN_STREAM("part " << part_name << " not in inventory!");
            } else { //if here, have found part in inventory; grab it
                //populate a "Part" message
                pick_part.location = bin_num; // don't really care about the rest for move_cruise fnc
                pick_part.name = part_name.c_str();
                pick_part.pose = bin_part_pose_stamped;
                ROS_INFO_STREAM("found part in bin " << bin_num << " at location " << bin_part_pose_stamped << endl);
                if (!pick_part_fnc(pick_part)) {
                    ROS_WARN("-----------pick_part failed--------------");
                } else {
                    ros::spinOnce(); //update view of box1
                    //populate a destination Part object
                    //compute the destination location:
                    //  geometry_msgs::PoseStamped ShipmentFiller::compute_stPose_part_in_box_wrt_world(geometry_msgs::Pose pose_wrt_box,geometry_msgs::PoseStamped box_pose_wrt_world) {
                    pose_wrt_box = product.pose;
                    box_placement_location_code = get_box_placement_location_code(pose_wrt_box);
                    ROS_INFO("part destination placement code: %d ",(int) box_placement_location_code);
                    //destination_close_to_near_box_edge = test_pose_close_to_near_box_edge(pose_wrt_box);
                    box_part_pose_stamped = compute_stPose_part_in_box_wrt_world(pose_wrt_box, box_1_stamped_pose_);
                    //correct the height of the box, which is known:
                    ROS_INFO_STREAM("want to place part in box at world coords:"<<box_part_pose_stamped<<endl);
                

                    place_part = pick_part;
                    //FIX ME: should use RobotMove codes for destination
                    //???jspace cruise pose expects location code from PART message!!
                    //reconcile this!!
                    //Part::QUALITY_SENSOR_1
                    place_part.location = inventory_msgs::Part::QUALITY_SENSOR_1;  //inventory_msgs::Part::QUALITY_SENSOR_1; //CODE FOR Q1 LOCATION; 
                        
                    //place_part.location = robot_move_as::RobotMoveGoal::Q1_DROPOFF_UNKNOWN;  //inventory_msgs::Part::QUALITY_SENSOR_1; //CODE FOR Q1 LOCATION; 
                    place_part.pose = box_part_pose_stamped;
                    place_part.box_placement_location_code = box_placement_location_code; //choose wrist near vs wrist far soln
                    ROS_INFO_STREAM("part destination: " << place_part << endl);
                    //bool release_placed_part(double timeout=2.0);
                    //move_status = place_part_no_release(place_part);
                    ROS_INFO("calling place_part_no_release");
                    if (!place_part_no_release(place_part)) {
                        ROS_WARN("place_part failed");
                    } else { //release the part
                        ROS_WARN("SHOULD CHECK GOOD/BAD INSPECTION NOW");
                        ROS_WARN("IF INSPECTION GOOD, CHECK PART LOCATION AND FIX, IF NECESSARY");
                    
                        ROS_INFO("shipment-filler requesting release placed  part...");
                        if (!robotBehaviorInterface.release()) {
                            ROS_WARN("trouble releasing part");

                        }
                    }

                }


            }
            orderManager.update_inventory();
        }
        cout<<"enter 1: ";
        cin>>ans;
        return true;
    }
     */

    /*
    bool ShipmentFiller::pick_part_fnc(inventory_msgs::Part part) {
        bool robot_move_ok;
        ROS_INFO("moving to bin cruise pose ");
        robot_move_ok = robotBehaviorInterface.move_cruise_pose(part,4.0);
        ros::Duration(2.0).sleep();
        if (!robot_move_ok) {
            ROS_WARN("problem moving to bin cruise pose");
            return false;
        }
        ROS_INFO("acquiring part");
        robot_move_ok = robotBehaviorInterface.pick(part);
        //this ends up in hover pose
        if (!robot_move_ok) {
            ROS_WARN("problem picking part");
            return false;
        }

        ROS_INFO("moving to bin cruise pose");
        robot_move_ok = robotBehaviorInterface.move_cruise_pose(part,4.0);
        ros::Duration(2.0).sleep();
        if (!robot_move_ok) {
            ROS_WARN("problem moving to bin cruise pose");
            return false;
        }

        return true;
    }
     */
    //    geometry_msgs::Pose pose_wrt_box;
    /*
    bool ShipmentFiller::test_pose_close_to_near_box_edge(geometry_msgs::Pose pose_wrt_box) {
    //FINISH ME!!!  default to true, so will choose wrist "far", if that's an option
      //w/rt box, if x<0 and y>0, pose is close to robot and left side w/rt robot
        double x = pose_wrt_box.position.x;
        double y = pose_wrt_box.position.y;
        if ((x<0)&&(y>0)) return true;
      //maybe good enough??
        return false;
    }
     */
    //box placement codes:
    //int8 PART_FAR_RIGHT = 30
    //int8 PART_FAR_LEFT = 31
    //int8 PART_NEAR_RIGHT = 32
    //int8 PART_NEAR_LEFT = 33
    //pose_of_part_wrt_box:
    //x<0 is "near"  x>0 is "far"
    //y<0 is "right", y>0 is "left"
    /*
    unsigned short int ShipmentFiller::get_box_placement_location_code(geometry_msgs::Pose pose_wrt_box) {
        double x = pose_wrt_box.position.x;
        double y = pose_wrt_box.position.y;
        if ((x<=0)&&(y<=0)) { 
            return robot_move_as::RobotMoveGoal::PART_NEAR_RIGHT;
        }
        if ((x<0)&&(y>0)) { 
            return robot_move_as::RobotMoveGoal::PART_NEAR_LEFT;
        }    
         if ((x>0)&&(y<=0)) { 
            return robot_move_as::RobotMoveGoal::PART_FAR_RIGHT;
        }

        return robot_move_as::RobotMoveGoal::PART_FAR_LEFT;
    }
     */
    /*
    bool ShipmentFiller::test_pose_close_to_near_box_edge(geometry_msgs::Pose pose_wrt_box) {
    //FINISH ME!!!  default to true, so will choose wrist "far", if that's an option
      //w/rt box, if x<0 and y>0, pose is close to robot and left side w/rt robot
        double x = pose_wrt_box.position.x;
        double y = pose_wrt_box.position.y;
        if ((x<0)&&(y>0)) return true;
      //maybe good enough??
        return false;
    }
     */

    //given a part name, see if it is in inventory AND is IK reachable (pickable)

    bool ShipmentFiller::select_part_from_inventory(std::string part_name, int &bin_num, geometry_msgs::PoseStamped &part_pose) {
        orderManager.get_inventory(inventory_msg_);
        int part_id = name_to_part_id_mappings[part_name];
        int num_parts_avail = inventory_msg_.inventory[part_id].part_stamped_poses.size();
        if (num_parts_avail < 1) return false;
        inventory_msgs::Part part; //object to populate for use w/ robotMove 

        //get the first available part:
        for (int ipart = 0; ipart < num_parts_avail; ipart++) {
            bin_num = inventory_msg_.inventory[part_id].bins[ipart];
            part_pose = inventory_msg_.inventory[part_id].part_stamped_poses[ipart];
            //convert to a "Part" object
            part.location = bin_num;
            part.pose = part_pose;
            part.name = part_name.c_str();
            //check if this part is pickable:
            //    unsigned short int is_pickable(const robot_move_as::RobotMoveGoalConstPtr &goal);
            //bool RobotMove::test_is_pickable(Part part) {
            //if (robotBehaviorInterface.test_is_pickable(part)) {
            //    return true; //found a viable candidate; stop here, returning arg values
            //}
        }
        //ROS_WARN_STREAMER("no "<<part_name<<" parts are both in stock and pickable");
        //return false;
        return true;

    }


    //assumes starts from bin cruise pose

    /*
    bool ShipmentFiller::place_part_no_release(inventory_msgs::Part destination_part) {
        bool robot_move_ok;
        //ROS_INFO("moving to box-fill cruise pose ");
        //robot_move_ok = robotMove.move_cruise_pose(destination_part,4.0);
        //ros::Duration(2.0).sleep();
        //if (!robot_move_ok) {
        //    ROS_WARN("problem moving to box-fill cruise pose");
        //    return false;
        //}
        ROS_INFO("attempting part placement; timeout at %f sec",MAX_ACTION_SERVER_WAIT_TIME);
        robot_move_ok = robotBehaviorInterface.place_part_no_release(destination_part, MAX_ACTION_SERVER_WAIT_TIME);
        ROS_INFO("robotBehaviorInterface.place_part_no_release() returned with robot_move_ok = %d",robot_move_ok);
        //ros::Duration(5.0).sleep();
        return robot_move_ok;
    }
     */

    bool ShipmentFiller::replace_faulty_parts_inspec1(osrf_gear::Shipment shipment) {
        inventory_msgs::Part bad_part;
        if(get_bad_part_Q1(bad_part)){   
        	if(!robotBehaviorInterface.discard_grasped_part(bad_part)) {
                ROS_INFO("Unable to discard");
                return 0;
            }

            ROS_INFO("successfully discarded faulty part from Q1");
            return true;
            
        }
        
            ROS_INFO(" The currently grasped part satisfies quality requirements");
            return true;
        }
    

    bool ShipmentFiller::replace_faulty_parts_inspec2(osrf_gear::Shipment shipment) {
     
        inventory_msgs::Part bad_part;
        while(get_bad_part_Q2(bad_part)){
            if(!robotBehaviorInterface.pick_part_from_box(bad_part)){
                ROS_INFO("unable to pick faulty part at inspection2"); //Making it very verbose
               return 0;
            }
            if(!robotBehaviorInterface.discard_grasped_part(bad_part)) {
                ROS_INFO("Unable to discard");
                return 0;
            }

            ROS_INFO("successfully discarded faulty part from Q2");
            ros::spinOnce();
        }
        
            ROS_INFO("No (more) faulty parts");
            return 1;
    }
    

//RECHECK BELOW FNC

    bool ShipmentFiller::adjust_shipment_part_locations(vector<osrf_gear::Model> desired_models_wrt_world) { 
        vector<osrf_gear::Model> misplaced_models_actual_coords_wrt_world,misplaced_models_desired_coords_wrt_world;    
        bool all_success=true;
        if(boxInspector.post_dropoff_check(desired_models_wrt_world,misplaced_models_desired_coords_wrt_world,misplaced_models_actual_coords_wrt_world)){ //COULD USE WHILE INSTEAD! OR MAYBE A FOR LOOP TO CHECK TWICE MAX!
        for(int i=0;i<misplaced_models_actual_coords_wrt_world.size();i++) {
            bool go_on=true;
            inventory_msgs::Part sourcePart,destinationPart;
            unsigned long int des_location=BOX_INSPECTION1_LOCATION_CODE;
            model_to_part(misplaced_models_actual_coords_wrt_world[i],sourcePart,des_location);
            model_to_part(misplaced_models_desired_coords_wrt_world[i],destinationPart,des_location);
            //need to pick first, pick and go to hover. maybe could use pick-adjust-place?
            if(!robotBehaviorInterface.pick_part_from_box(sourcePart,2)) {
            	ROS_INFO("unable to repick misplaced part");
            	all_success = false;
            }
            if(go_on) {
            if(!robotBehaviorInterface.adjust_part_location_no_release(sourcePart,destinationPart,2)) {
                ROS_INFO("cannot adjust, maybe pick first?");
                all_success=false;
            }
            robotBehaviorInterface.release_and_retract();
            }
        }
    }
        else{return 0;}  //TO USE WITH IF ON LINE 683
    
    return all_success;
}

	bool ShipmentFiller::adjust_part_location_before_dropoff(inventory_msgs::Part part) {
		osrf_gear::Model model_actual_coords, model_desired_coords;
		if(boxInspector.pre_dropoff_check(part, model_actual_coords,model_desired_coords)) {
			inventory_msgs::Part sourcePart,destinationPart;
			model_to_part(model_actual_coords,sourcePart);
			model_to_part(model_desired_coords,destinationPart);
			if(!robotBehaviorInterface.adjust_part_location_no_release(sourcePart,destinationPart,2)) {
				ROS_INFO("Failed to adjust part location");
			}
			else {
				ROS_INFO("successfully adjusted part");
				return 1;
			}
		}
		else {
			ROS_INFO("No need to adjust");
			return 1;
		}

	}
		



    bool ShipmentFiller::correct_dropped_part(osrf_gear::Shipment shipment) {
        vector<osrf_gear::Model> desired_models_wrt_world,satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,orphan_models_wrt_world;
        boxInspector.compute_shipment_poses_wrt_world(shipment,box_1_stamped_pose_,desired_models_wrt_world);
        if(!boxInspector.update_inspection(desired_models_wrt_world,satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,orphan_models_wrt_world)) {
        	return 0;
        }
        bool success;
        if(misplaced_models_actual_coords_wrt_world.size()==0){
            ROS_INFO("cant find dropped part");
            success=false; //or true depending on whether the fnc is only called when neccessary
        }
        else{
            for(int ipart=0;ipart<misplaced_models_actual_coords_wrt_world.size();ipart++) {
                inventory_msgs::Part part_to_retrieve,part_corrected;
                model_to_part(misplaced_models_actual_coords_wrt_world[ipart],part_to_retrieve);
                if(!robotBehaviorInterface.pick_part_from_box(part_to_retrieve)) {
                    ROS_INFO("error picking up dropped part");
                    ipart++;
                    success=false;
                }
                else {
                    model_to_part(misplaced_models_desired_coords_wrt_world[ipart],part_corrected);
                    if(!robotBehaviorInterface.place_part_in_box_with_release(part_corrected)) {
                        ROS_INFO("Cannot place part where desired");
                        ipart++;
                        success=false;
                    }
                    else {
                        ROS_INFO("corrected part %d",ipart);
                        ipart++;  
                        success=true;          
                    }
                }
            }
        }
        return success;
    }

       
    void ShipmentFiller::set_drone_shipment_name(osrf_gear::Shipment shipment) {
        droneControl_.request.shipment_type = shipment.shipment_type;
    }

    bool ShipmentFiller::report_shipment_to_drone() {
        //rosservice call /ariac/drone "shipment_type: order_0_shipment_0"
        //droneControl_.shipment_type =  SHOULD ALREADY BE SET
        ROS_INFO("informing drone of ready shipment");
        //rosservice call /ariac/drone "shipment_type: order_0_shipment_0"
        droneControl_.response.success = false;
        while (!droneControl_.response.success) {
            drone_client_.call(droneControl_);
            ros::Duration(0.5).sleep();
        }
        return true;
    }

    bool ShipmentFiller::advance_shipment_on_conveyor(int location_code) {
        //$ rosservice call /ariac/conveyor/control "power: 100"
        ROS_INFO("advancing conveyor to location code %d", location_code);
        bool conveyor_ok = false;
        double time=0;
        double dt = 0.5;
        double CONVEYOR_ADVANCE_MAX_TIME = 40.0;
        double CONVEYOR_ADVANCE_BOX_Q1 = 20.0;
        switch (location_code) {
            case BOX_INSPECTION1_LOCATION_CODE:
                time=0;
                while (!conveyor_ok) {
                    conveyor_client_.call(conveyor_svc_msg_GO_);
                    conveyor_ok = conveyor_svc_msg_GO_.response.success;
                    ros::spinOnce();
                }
                ROS_INFO("conveyor commanded to move");
                if (!box_cam1_sees_box_) ROS_INFO("box not yet seen at cam1 station");
                while ((!box_cam1_sees_box_) && (time<CONVEYOR_ADVANCE_BOX_Q1)) {
                    ROS_INFO("box not yet seen at cam1 station");
                    conveyor_client_.call(conveyor_svc_msg_GO_);
                    ros::spinOnce();
                    ros::Duration(dt).sleep();
                    time+=dt;
                }
                ROS_INFO("now see box at box_cam_1_dist_to_go_ = %f", box_cam_1_dist_to_go_);
                while (box_cam_1_dist_to_go_ > 0) {
                    ros::spinOnce();
                    ros::Duration(0.1).sleep();
                }
                ROS_INFO("stopping conveyor");
                conveyor_client_.call(conveyor_svc_msg_STOP_);
                ROS_INFO("now see box at box_cam_1_dist_to_go_ = %f", box_cam_1_dist_to_go_);
                return true;
                break;

            case BOX_INSPECTION2_LOCATION_CODE:
                ROS_INFO("advancing box to first inspection station at box cam2");
                conveyor_client_.call(conveyor_svc_msg_GO_);
                if (!box_cam2_sees_box_) ROS_INFO("box not yet seen at inspection 1 station");
                while (!box_cam2_sees_box_) {
                    ROS_INFO("box not yet seen at cam2 station");
                    ros::spinOnce();
                    ros::Duration(0.5).sleep();
                }
                ROS_INFO("now see box at box_cam_2_dist_to_go_ = %f", box_cam_2_dist_to_go_);
                while (box_cam_2_dist_to_go_ > 0) {
                    ros::spinOnce();
                    ros::Duration(0.1).sleep();
                }
                ROS_INFO("stopping conveyor");
                conveyor_client_.call(conveyor_svc_msg_STOP_);
                ROS_INFO("now see box at box_cam_2_dist_to_go_ = %f", box_cam_2_dist_to_go_);
                return true;
                break;

            case DRONE_DOCK_LOCATION_CODE:
                ROS_INFO("advancing box to drone depot");
                conveyor_client_.call(conveyor_svc_msg_GO_);
                time=0;
                while ((!drone_depot_sensor_sees_box_)&&(time<CONVEYOR_ADVANCE_MAX_TIME)) {
                    ros::Duration(dt).sleep();
                    time+=dt;
                    ros::spinOnce();
                    ROS_INFO("scanner does  not see box at drone depot");
                }
                ROS_INFO("prox sensor sees box!");
                conveyor_client_.call(conveyor_svc_msg_STOP_);
                return true;
                break;

            default: ROS_WARN("advance_shipment_on_conveyor: location code not recognized");
                return false;
                break;
        }

        return true;
    }

//class BinInventory is constructed to do the following:
//execute binInventory.update() to force a refresh from all installed bin cameras;
//data from these cameras refresh the structure "inventory" from scratch

//access the inventory with functions including:
// num_parts(part_name): returns the number of parts, with part name as arg
// find_part(part_name,&bin,&pose): fills in the bin# and pose of first part in the list
// find_part(part_name,part_n,&bin,&pose): fills in bin# and pose of n'th part in the list

//must edit this file to add more cameras, including subscriber init and callback fnc

#include<bin_inventory/bin_inventory.h>

//note:  must edit constructor to add cameras
//BinInventory(ros::NodeHandle* nodehandle);

BinInventory::BinInventory(ros::NodeHandle* nodehandle) : nh_(*nodehandle) { //constructor
    ROS_INFO("in class constructor of BinInventory");
    ROS_INFO("resizing vectors");
    num_part_types_ = NUM_PART_TYPES; //synonym
    logicalCamDataVec_.resize(NUM_CAMERAS+1);
    bin_camera_triggers_.resize(NUM_CAMERAS+1);
    camera_to_bin_mapping_.resize(NUM_CAMERAS+1);
    fillCamToBinMapping();  //not used?
    ROS_INFO("initializing triggers");
    if (NUM_CAMERAS<5) {
        int ans;
        ROS_WARN("bin_inventory.cpp is ignoring bin5;edit header file NUM_CAMERAS=5 to change this");
        //ROS_WARN("enter 1 to acknowledge: ");
        //cin>>ans;
    }
    for (int i = 0; i < NUM_CAMERAS; i++) bin_camera_triggers_[i] = false;

    ROS_INFO("initializing inventory");
    initializeInventory();

    initializeSubscribers();
    
    update();
    ROS_INFO("initial inventory: ");
    print_inventory_msg();

    //initializePartMappings(); //not working
}

//EDIT THIS TO ADD CAMERAS and reconcile with config file
//hmm...looks like I ended up not using this
void BinInventory::fillCamToBinMapping() {
    camera_to_bin_mapping_[1] = 1; // bin0 for cam0? no cam0 present
    camera_to_bin_mapping_[2] = 2; //cam1 is over bin1 for qual1 config
    camera_to_bin_mapping_[3] = 3; //
    camera_to_bin_mapping_[4] = 4;
    camera_to_bin_mapping_[5] = 5;    
}

//EDIT THIS TO ADD CAMERAS:

void BinInventory::initializeSubscribers() {

    ROS_INFO("Initializing Subscribers");

    logical_camera_1_subscriber_ = nh_.subscribe("/ariac/logical_camera_1", 1,
            &BinInventory::logical_camera_1_callback, this);
    logical_camera_2_subscriber_ = nh_.subscribe("/ariac/logical_camera_2", 1,
            &BinInventory::logical_camera_2_callback, this);
    logical_camera_3_subscriber_ = nh_.subscribe("/ariac/logical_camera_3", 1,
            &BinInventory::logical_camera_3_callback, this);
    logical_camera_4_subscriber_ = nh_.subscribe("/ariac/logical_camera_4", 1,
            &BinInventory::logical_camera_4_callback, this);
    logical_camera_5_subscriber_ = nh_.subscribe("/ariac/logical_camera_5", 1,
            &BinInventory::logical_camera_5_callback, this);    
    //ADD MORE CAMERAS HERE; ALSO ADD CORRESPONDING CALLBACK FNCSs
}

//when callback wakes up, it copies received data to logicalCamDataVec_


void BinInventory::logical_camera_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    int cam_num = 1; //change this number for each new callback fnc
    if (!bin_camera_triggers_[cam_num]) {
        //ROS_INFO("in callback for cam  %d, triggered",cam_num);
        update_camera_data(cam_num, image_msg);
    }
}

void BinInventory::logical_camera_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    int cam_num = 2; //change this number for each new callback fnc
    //   ROS_INFO("in callback for cam  %d",cam_num);
    if (!bin_camera_triggers_[cam_num]) {
        update_camera_data(cam_num, image_msg);
    }
}

void BinInventory::logical_camera_3_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    int cam_num = 3; //change this number for each new callback fnc
    //   ROS_INFO("in callback for cam  %d",cam_num);
    if (!bin_camera_triggers_[cam_num]) {
        update_camera_data(cam_num, image_msg);
    }
}

void BinInventory::logical_camera_4_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    int cam_num = 4; //change this number for each new callback fnc
    //   ROS_INFO("in callback for cam  %d",cam_num);
    if (!bin_camera_triggers_[cam_num]) {
        update_camera_data(cam_num, image_msg);
    }
}

void BinInventory::logical_camera_5_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    int cam_num = 5; //change this number for each new callback fnc
    //   ROS_INFO("in callback for cam  %d",cam_num);
    if (!bin_camera_triggers_[cam_num]) {
        update_camera_data(cam_num, image_msg);
    }
}


void BinInventory::update_camera_data(int cam_num, const osrf_gear::LogicalCameraImage::ConstPtr image_msg) {
    if (cam_num < NUM_CAMERAS+1) {
        osrf_gear::LogicalCameraImage image_data;
        copy_logical_camera_data(image_msg, image_data);
        //ROS_INFO_STREAM("image  data: " << image_data << std::endl);
        logicalCamDataVec_[cam_num] = image_data;
        bin_camera_triggers_[cam_num] = true;
        ROS_INFO("CAMERA %d data has been updated", cam_num);
    } else {
        //ROS_WARN("can't update camera %d; does  not exist!", cam_num);
    }
}



//perform a copy of data received by logical camera subscriber 

void BinInventory::copy_logical_camera_data(const osrf_gear::LogicalCameraImage::ConstPtr image_msg,
        osrf_gear::LogicalCameraImage &image_data) {
    //FIX ME!!
    // ROS_INFO_STREAM(
    //        "Logical camera: '" << image_msg->models.size() << "' objects.");
    image_data = *image_msg; //does this work?
}

//std::vector<PartInventory> inventory_;

void BinInventory::initializeInventory() {
    //inventory_.resize(num_part_types_+1); //part_id=0 is not valid
    //clear_inventory();
    //inventory_msg_.resize(NUM_PART_TYPES+1);
    clear_inventory_msg();
}

bool BinInventory::all_cameras_updated() {
    bool ready = bin_camera_triggers_[1];
    for (int i = 2; i < NUM_CAMERAS+1; i++)
        ready = ready && (bin_camera_triggers_[i]);
    return ready;
}

bool BinInventory::update() {
    //reset subscriber triggers: leave out non-existent cam0 
    ROS_INFO(" ");
    ROS_INFO("updating inventory");
    bin_camera_triggers_[0] = true;
    for (int i = 1; i <= NUM_CAMERAS; i++) bin_camera_triggers_[i] = false;
    double dt= 0.05;
    double timer = 0.0;
    while (!all_cameras_updated()&&(timer<BIN_INVENTORY_TIMEOUT)) {
        ros::spinOnce();
        ros::Duration(dt).sleep();
        timer+=dt;
    }
    if (timer>=BIN_INVENTORY_TIMEOUT) {
        ROS_WARN("could not update inventory!");
        return false;
    }
    //if here, DID  receive input from every bin camera
    //clear out the inventory and re-fill from scratch:
    //clear_inventory();
    clear_inventory_msg();
    inventory_msg_.inventory.resize(NUM_PART_TYPES+1);
    

    //should have all camera data now stored in logicalCamDataVec_
    //walk through this data and update inventory
    osrf_gear::LogicalCameraImage image_data;
    geometry_msgs::PoseStamped stPose_part_wrt_world;
    for (int cam_num = 1; cam_num < NUM_CAMERAS+1; cam_num++) {
        image_data = logicalCamDataVec_[cam_num];
        int num_models = image_data.models.size();
        ROS_INFO("cam %d saw %d models", cam_num, num_models);
        if (num_models > 0) {
            geometry_msgs::Pose cam_pose = image_data.pose;
            geometry_msgs::Pose part_pose;
            int bin_num = camera_to_bin_mapping_[cam_num];
            
            for (int imodel = 0; imodel < num_models; imodel++) {
                std::string name(image_data.models[imodel].type);
                // cout << "model name: " << name << endl;
                int part_id = name_to_part_id_mappings[name];
                if (part_id == 0) {
                    ROS_WARN("model name %s is not in mappings! ", name.c_str());
                }
                else {
                  // ROS_INFO("part ID = %d", part_id);
                  //  ROS_INFO("entering inventory for cam %d, model %d",cam_num,imodel);
                  part_pose = image_data.models[imodel].pose;
                  //((inventory_[part_id]).part_stamped_poses).push_back();
                  //inventory_[part_id].bins.push_back(bin_num);
                  inventory_msg_.inventory[part_id].bins.push_back(bin_num);
                  stPose_part_wrt_world= compute_stPose(cam_pose,part_pose);
                  //inventory_[part_id].part_stamped_poses.push_back(stPose_part_wrt_world);
                  inventory_msg_.inventory[part_id].part_stamped_poses.push_back(stPose_part_wrt_world);                  
                }
            }
        }

    }
    ROS_INFO("inventory updated");
    return true;
    //print_inventory();
}

geometry_msgs::PoseStamped BinInventory::compute_stPose(geometry_msgs::Pose cam_pose,geometry_msgs::Pose part_pose) {
    
    Eigen::Affine3d cam_wrt_world,part_wrt_cam,part_wrt_world;
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

int BinInventory::num_parts(std::string name) {
    int part_id = name_to_part_id_mappings[name];
    int num_parts_in_inventory = inventory_msg_.inventory[part_id].part_stamped_poses.size();
    return num_parts_in_inventory;
} 

int BinInventory::num_parts(int part_id) {
    int num_parts_in_inventory = inventory_msg_.inventory[part_id].part_stamped_poses.size();//THERE SEEMS TO BE A PROBLEM HERE!
    return num_parts_in_inventory;
}

int BinInventory::num_parts(inventory_msgs::Inventory inventory, int part_id) {
    int num_parts_in_inventory = inventory.inventory[part_id].part_stamped_poses.size();
    return num_parts_in_inventory;
}

int BinInventory::num_parts(inventory_msgs::Inventory inventory, std::string name) { //return number of parts
    int part_id = name_to_part_id_mappings[name];
    int num_parts_in_inventory = inventory.inventory[part_id].part_stamped_poses.size();
    return num_parts_in_inventory;
}

void BinInventory::counts_all_part_types(std::vector<int> &parts_counts) {
    parts_counts.resize(NUM_PART_TYPES+1,0);
    for (int part_id=1;part_id<NUM_PART_TYPES+1;part_id++) {
        parts_counts[part_id]= inventory_msg_.inventory[part_id].bins.size();  //bins.size(); //same as
    }
}

//select a part at random from  among available parts of specified type;
//return the partnum, so it can be removed from  inventory manually, if desired
/*
bool BinInventory::find_part(std::string part_name,int &bin_num, geometry_msgs::PoseStamped &part_pose, int &partnum) {
  int part_id = name_to_part_id_mappings[part_name];
  int num_parts_avail = num_parts(part_id);
  if (num_parts_avail<1) return false;

  //select a part at random, to avoid re-trying the same part repeatedly
  partnum = 0;
  if (num_parts_avail>1) partnum = rand() % (num_parts_avail-1);
  bin_num =   inventory_msg_.inventory[part_id].bins[partnum];
  part_pose = inventory_msg_.inventory[part_id].part_stamped_poses[partnum];
  ROS_INFO_STREAM("found part "<<part_name<<" in bin "<<bin_num<<" at pose "<<part_pose<<endl);
  return true;

}
 * */
//as above,  but also populates a Part object
/*
bool BinInventory::find_part(std::string part_name,inventory_msgs::Part &pick_part, int &partnum) {
    geometry_msgs::PoseStamped part_pose;
    unsigned short int bin_num;
    find_part(part_name,bin_num,part_pose, partnum);
    //populate a Part  object:
    pick_part.location = bin_num;
    pick_part.pose = part_pose;
    pick_part.name = part_name.c_str();
  return true;

}
   */           
bool BinInventory::find_part(inventory_msgs::Inventory current_inventory,std::string part_name,inventory_msgs::Part &pick_part, int &partnum) {
  int part_id = name_to_part_id_mappings[part_name];
  ROS_INFO("debug: part id: %d", part_id );
  int num_parts_avail = num_parts(current_inventory,part_id); //made a change here that should definitely fix the segfault bug
  ROS_INFO("debug: num parts available: %d", num_parts_avail);
  if (num_parts_avail<1) return false;

  //select a part at random, to avoid re-trying the same part repeatedly
  partnum = 0;
  if (num_parts_avail>4) {
        ROS_WARN("DEBUG:  choose part #4 among inventory options");  
      partnum = 4;
  }
  else if (num_parts_avail>1) { 
      partnum = rand() % (num_parts_avail-1);
  }
  
  int bin_num =   current_inventory.inventory[part_id].bins[partnum];
  ROS_INFO("DEBUG: bin_num: %d", bin_num);
  geometry_msgs::PoseStamped part_pose = current_inventory.inventory[part_id].part_stamped_poses[partnum];
  ROS_INFO_STREAM("found part "<<part_name<<" in bin "<<bin_num<<" at pose "<<part_pose<<endl);
    //populate a Part  object:
    pick_part.location = bin_num;
    pick_part.pose = part_pose;
    pick_part.name = part_name.c_str();
  return true;
    
}
              
              

bool BinInventory::remove_part_from_inventory(int part_id, int partnum) {
    if ((part_id<1)||(part_id>NUM_PART_TYPES)) {
        ROS_WARN("invalid part_id!");
        return false; 
    }
    if (partnum>= num_parts(part_id)) {
        ROS_WARN("invalid partnum...not that many parts!");
        return false;
    }
    //OK--should be safe to do this:
  inventory_msg_.inventory[part_id].bins.erase(inventory_msg_.inventory[part_id].bins.begin()+partnum);
  inventory_msg_.inventory[part_id].part_stamped_poses.erase(inventory_msg_.inventory[part_id].part_stamped_poses.begin()+partnum);
  return true;
}

bool BinInventory::remove_part_from_inventory( int part_id, int partnum, inventory_msgs::Inventory &inventory) {
    if ((part_id<1)||(part_id>NUM_PART_TYPES)) {
        ROS_WARN("invalid part_id!");
        return false; 
    }
    int num_parts = inventory.inventory[part_id].bins.size(); //num_parts(inventory,part_id); does the same thing, really. Could change for uniformity
    if (partnum>= num_parts) {
        ROS_WARN("invalid partnum...not that many parts in provided inventory!");
        return false;
    }
    //OK--should be safe to do this:
  inventory.inventory[part_id].bins.erase(inventory.inventory[part_id].bins.begin()+partnum);
  inventory.inventory[part_id].part_stamped_poses.erase(inventory.inventory[part_id].part_stamped_poses.begin()+partnum);
  return true;    
    
}


/*bool OrderManager::delete_from_order_queue(int order_index, std::vector<osrf_gear::Order> &order_vec)  {
    //e.g., myvector.erase (myvector.begin()+5);
    order_vec.erase(order_vec.begin()+order_index); 
}*/



void BinInventory::clear_inventory_msg() {    
    //ROS_INFO("clearing inventory message: ");
    inventory_msg_.inventory.clear();
}

void BinInventory::print_inventory_msg() {
    ROS_INFO_STREAM("inventory: "<<inventory_msg_<<endl);
}

void BinInventory::get_inventory(inventory_msgs::Inventory &inventory_msg) {
    inventory_msg = inventory_msg_;  //does this work?
}


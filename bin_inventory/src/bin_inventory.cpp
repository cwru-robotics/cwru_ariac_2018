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
    logicalCamDataVec_.resize(NUM_CAMERAS);
    bin_camera_triggers_.resize(NUM_CAMERAS);
    camera_to_bin_mapping_.resize(NUM_CAMERAS);
    fillCamToBinMapping();
    ROS_INFO("initializing triggers");
    for (int i = 0; i < NUM_CAMERAS; i++) bin_camera_triggers_[i] = false;

    ROS_INFO("initializing inventory");
    initializeInventory();

    initializeSubscribers();
    
    update();
    ROS_INFO("initial inventory: ");
    print_inventory();

    //initializePartMappings(); //not working
}

//EDIT THIS TO ADD CAMERAS:

void BinInventory::fillCamToBinMapping() {
    camera_to_bin_mapping_[0] = 0; // fix this--bin0 for cam0? no cam0 present
    camera_to_bin_mapping_[1] = 3; //cam1 is over bin3 for exaxmple ARIAC config
    camera_to_bin_mapping_[2] = 0; //cam2 is actually over conveyor for ARIAC config
}

//EDIT THIS TO ADD CAMERAS:

void BinInventory::initializeSubscribers() {

    ROS_INFO("Initializing Subscribers");

    logical_camera_0_subscriber_ = nh_.subscribe("/ariac/logical_camera_0", 1,
            &BinInventory::logical_camera_0_callback, this);

    logical_camera_1_subscriber_ = nh_.subscribe("/ariac/logical_camera_1", 1,
            &BinInventory::logical_camera_1_callback, this);

    logical_camera_2_subscriber_ = nh_.subscribe("/ariac/logical_camera_2", 1,
            &BinInventory::logical_camera_2_callback, this);

    //ADD MORE CAMERAS HERE; ALSO ADD CORRESPONDING CALLBACK FNCSs
}

//when callback wakes up, it copies received data to logicalCamDataVec_

void BinInventory::logical_camera_0_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    int cam_num = 0; //change this number for each new callback fnc
    //if trigger  has been set, update the camera data
    if (!bin_camera_triggers_[cam_num]) {
        update_camera_data(cam_num, image_msg);
    }
}

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

void BinInventory::update_camera_data(int cam_num, const osrf_gear::LogicalCameraImage::ConstPtr image_msg) {
    if (cam_num < NUM_CAMERAS) {
        osrf_gear::LogicalCameraImage image_data;
        copy_logical_camera_data(image_msg, image_data);
        //ROS_INFO_STREAM("image  data: " << image_data << std::endl);
        logicalCamDataVec_[cam_num] = image_data;
        bin_camera_triggers_[cam_num] = true;
        ROS_INFO("CAMERA %d data has been updated", cam_num);
    } else {
        ROS_WARN("can't update camera %d; does  not exist!", cam_num);
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
    inventory_.resize(num_part_types_+1); //part_id=0 is not valid
    clear_inventory();
}

bool BinInventory::all_cameras_updated() {
    bool ready = bin_camera_triggers_[0];
    for (int i = 0; i < NUM_CAMERAS; i++)
        ready = ready && (bin_camera_triggers_[i]);
    return ready;
}

void BinInventory::update() {
    //reset subscriber triggers: leave out cam0 for test
    bin_camera_triggers_[0] = true;
    for (int i = 1; i < NUM_CAMERAS; i++) bin_camera_triggers_[i] = false;
    while (!all_cameras_updated()) {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    //clear out the inventory and re-fill from scratch:
    clear_inventory();

    //should have all camera data now stored in logicalCamDataVec_
    //walk through this data and update inventory
    osrf_gear::LogicalCameraImage image_data;
    geometry_msgs::PoseStamped stPose_part_wrt_world;
    for (int cam_num = 0; cam_num < NUM_CAMERAS; cam_num++) {
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
                int part_id = mappings[name];
                if (part_id == 0) {
                    ROS_WARN("model name %s is not in mappings! ", name.c_str());
                }
                else {
                  // ROS_INFO("part ID = %d", part_id);
                  //  ROS_INFO("entering inventory for cam %d, model %d",cam_num,imodel);
                  part_pose = image_data.models[imodel].pose;
                  //((inventory_[part_id]).part_stamped_poses).push_back();
                  inventory_[part_id].bins.push_back(bin_num);
                  stPose_part_wrt_world= compute_stPose(cam_pose,part_pose);
                  inventory_[part_id].part_stamped_poses.push_back(stPose_part_wrt_world);
                }
            }
        }

    }
    ROS_INFO("inventory updated");
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
    int part_id = mappings[name];
    int num_parts_in_inventory = inventory_[part_id].part_stamped_poses.size();
    return num_parts_in_inventory;
} 

int BinInventory::num_parts(int part_id) {
    int num_parts_in_inventory = inventory_[part_id].part_stamped_poses.size();
    return num_parts_in_inventory;
}

//clear out the inventory and re-fill from scratch:
void BinInventory::clear_inventory() {    
    ROS_INFO("clearing inventory: ");
    for (int ipart = 0; ipart < NUM_PART_TYPES; ipart++) {
        inventory_[ipart].part_stamped_poses.clear();
        inventory_[ipart].bins.clear();
    }
    //ROS_INFO("inventory after clearance: ");
    //print_inventory();
}

void BinInventory::print_inventory() {
    ROS_INFO("printing inventory");
    for (int part_id=1;part_id<NUM_PART_TYPES+1;part_id++) {
        int num_parts = inventory_[part_id].bins.size(); //same as part_stamped_poses.size()
        ROS_INFO("part_id = %d; num parts = %d",part_id,num_parts);
        for (int i=0;i<num_parts;i++) {
            ROS_INFO_STREAM("ipart: "<<i<<" pose: "<<inventory_[part_id].part_stamped_poses[i]<<endl);
        } 
    }
    ROS_INFO("  ");
    
}

void BinInventory::get_inventory(std::vector<PartInventory> &inventory) {
    inventory = inventory_; // does this work?
}
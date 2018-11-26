//box_inspector.cpp implementation of class/library
#include <box_inspector/box_inspector.h>
//#include "box_inspector_fncs.cpp" //more code, outside this file
#include "box_inspector_fncs.cpp" //more code, outside this file
#include <math.h>
using namespace std;

BoxInspector::BoxInspector(ros::NodeHandle* nodehandle) : nh_(*nodehandle) { //constructor
    //set up camera subscriber:
    box_camera_subscriber_ = nh_.subscribe("/ariac/box_camera_1", 1, &BoxInspector::box_camera_callback, this);
    got_new_snapshot_ = false; //trigger to get new snapshots

    //  geometry_msgs::PoseStamped NOM_BOX1_POSE_WRT_WORLD,NOM_BOX2_POSE_WRT_WORLD;
    // assign hard-coded nominal vals for boxes at Q1 and Q1:
    //0.55, 0.61, 0.588; rpy = 0,0,0
    NOM_BOX1_POSE_WRT_WORLD.header.frame_id = "world";
    NOM_BOX1_POSE_WRT_WORLD.pose.position.x = 0.55;
    NOM_BOX1_POSE_WRT_WORLD.pose.position.y = 0.61;
    NOM_BOX1_POSE_WRT_WORLD.pose.position.z = 0.588;
    NOM_BOX1_POSE_WRT_WORLD.pose.orientation.x = 0.0;
    NOM_BOX1_POSE_WRT_WORLD.pose.orientation.y = 0.0;
    NOM_BOX1_POSE_WRT_WORLD.pose.orientation.z = 0.0;
    NOM_BOX1_POSE_WRT_WORLD.pose.orientation.w = 1.0;

    NOM_BOX2_POSE_WRT_WORLD = NOM_BOX1_POSE_WRT_WORLD;
    NOM_BOX2_POSE_WRT_WORLD.pose.position.y = 0.266;

    quality_sensor_1_subscriber_ = nh_.subscribe("/ariac/quality_control_sensor_1", 1,
            &BoxInspector::quality_sensor_1_callback, this);
    qual_sensor_1_sees_faulty_part_ = false;

    quality_sensor_2_subscriber_ = nh_.subscribe("/ariac/quality_control_sensor_2", 1,
            &BoxInspector::quality_sensor_2_callback, this);
    qual_sensor_2_sees_faulty_part_ = false;

}

void part_to_model(inventory_msgs::Part part, osrf_gear::Model &model) {
    model.type = part.name;
    model.pose = part.pose.pose;

}

void BoxInspector::quality_sensor_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    qual_sensor_1_image_ = *image_msg;
    //ROS_INFO("got Qsensor1 msg...");
    qual_sensor_1_sees_faulty_part_ = find_faulty_part_Q1(qual_sensor_1_image_, bad_part_Qsensor1_);
    got_new_Q1_image_ = true;
}

void BoxInspector::quality_sensor_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    qual_sensor_2_image_ = *image_msg;
    qual_sensor_2_sees_faulty_part_ = find_faulty_part_Q2(qual_sensor_2_image_, bad_part_Qsensor2_);
    got_new_Q2_image_ = true;
}

//note: this function returns only the FIRST faulty part found;
//but that should be suitable for our use

bool BoxInspector::find_faulty_part_Q1(const osrf_gear::LogicalCameraImage qual_sensor_image,
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

bool BoxInspector::find_faulty_part_Q2(const osrf_gear::LogicalCameraImage qual_sensor_image,
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

bool BoxInspector::get_bad_part_Q1(inventory_msgs::Part &bad_part) {
    got_new_Q1_image_ = false;
    double wait_time = 0;
    double dt = 0.1;
    while ((wait_time < QUALITY_INSPECTION_MAX_WAIT_TIME)&&!got_new_Q1_image_) {
        wait_time += dt;
        ros::spinOnce();
        ros::Duration(dt).sleep();
    }
    if (wait_time >= QUALITY_INSPECTION_MAX_WAIT_TIME) {
        ROS_WARN("timed  out waiting for quality inspection cam1");
        return false;
    }
    //if here, then got an update from Q1 cam:
    bad_part = bad_part_Qsensor1_;
    return qual_sensor_1_sees_faulty_part_;
}

bool BoxInspector::get_bad_part_Q2(inventory_msgs::Part &bad_part) {
    got_new_Q2_image_ = false;
    double wait_time = 0;
    double dt = 0.1;
    while ((wait_time < QUALITY_INSPECTION_MAX_WAIT_TIME)&&!got_new_Q2_image_) {
        wait_time += dt;
        ros::spinOnce();
        ros::Duration(dt).sleep();
    }
    if (wait_time >= QUALITY_INSPECTION_MAX_WAIT_TIME) {
        ROS_WARN("timed  out waiting for quality inspection cam2");
        return false;
    }
    //if here, then got an update from Q2 cam:
    bad_part = bad_part_Qsensor2_;
    return qual_sensor_2_sees_faulty_part_;
}

bool BoxInspector::find_orphan_parts(vector<osrf_gear::Model> desired_models_wrt_world, vector<osrf_gear::Model> &orphan_models) {

    vector<osrf_gear::Model> satisfied_models_wrt_world, misplaced_models_desired_coords_wrt_world, misplaced_models_actual_coords_wrt_world, missing_wrt_world;
    if (!update_inspection(desired_models_wrt_world, satisfied_models_wrt_world, misplaced_models_actual_coords_wrt_world, misplaced_models_desired_coords_wrt_world, missing_wrt_world, orphan_models)) {
        return 0;
    }
    if (orphan_models.size() == 0) {
        return 0;
    }
    return 1;
}

bool BoxInspector::find_missing_parts(vector<osrf_gear::Model> desired_models_wrt_world, vector<osrf_gear::Model> &missing_wrt_world) {
    vector<osrf_gear::Model> satisfied_models_wrt_world, misplaced_models_desired_coords_wrt_world, misplaced_models_actual_coords_wrt_world, orphan_models_wrt_world;
    if (!update_inspection(desired_models_wrt_world, satisfied_models_wrt_world, misplaced_models_actual_coords_wrt_world, misplaced_models_desired_coords_wrt_world, missing_wrt_world, orphan_models_wrt_world)) {
        return 0;
    }
    if (missing_wrt_world.size() == 0) {
        return 0;
    }
    return 1;
}

void BoxInspector::model_to_part(osrf_gear::Model model, inventory_msgs::Part &part, unsigned short int location) {
    part.name = model.type;
    part.pose.pose = model.pose;
    part.location = location; //by default
}

//obsolete; use 
//  bool get_grasped_part_pose_wrt_world(inventory_msgs::Part &observed_part);

bool BoxInspector::get_observed_part_pose(inventory_msgs::Part place_part, inventory_msgs::Part &observed_part) {
    if (!get_new_snapshot_from_box_cam()) return false; //give up if blackout

    int winner = 0;
    float max_ht = 0;
    int debug;
    bool found_a_candidate=false;
    geometry_msgs::PoseStamped grasped_pose_wrt_wrld;
    string grasped_part_name(place_part.name); 
        
    for (int i = 1; i < box_inspector_image_.models.size(); i++) {
        string model_name(box_inspector_image_.models[i].type);
        if (model_name==grasped_part_name) {
            found_a_candidate=true;
            grasped_pose_wrt_wrld = compute_stPose(box_inspector_image_.pose, box_inspector_image_.models[i].pose);
            if (grasped_pose_wrt_wrld.pose.position.z > max_ht) {
                max_ht = grasped_pose_wrt_wrld.pose.position.z;
                winner = i;
            }
        }
    }
    if (!found_a_candidate) return false; // certainly did not see intended grasped part
    
    grasped_pose_wrt_wrld = compute_stPose(box_inspector_image_.pose, box_inspector_image_.models[winner].pose);
    ROS_INFO_STREAM("winning model is:" << grasped_pose_wrt_wrld);

    model_to_part(box_inspector_image_.models[winner], observed_part);
    observed_part.pose = grasped_pose_wrt_wrld;
    return true; //have at least a candidate pose for grasped part; needs further vetting
}

bool BoxInspector::post_dropoff_check(vector<osrf_gear::Model> desired_models_wrt_world, vector<osrf_gear::Model> &misplaced_models_desired_coords, vector<osrf_gear::Model> &misplaced_models_actual_coords) {
    vector<osrf_gear::Model> satisfied_models_wrt_world, missing_models_wrt_world, orphan_models_wrt_world;
    if (!update_inspection(desired_models_wrt_world, satisfied_models_wrt_world, misplaced_models_actual_coords, misplaced_models_desired_coords, missing_models_wrt_world, orphan_models_wrt_world)) {
        return 0;
    }

    if (misplaced_models_desired_coords.size() == 0 || misplaced_models_actual_coords.size() == 0) {
        return 0;

    } else {

        return 1;
    }
}

bool BoxInspector::pre_dropoff_check(inventory_msgs::Part part, osrf_gear::Model misplaced_model_actual_coords, osrf_gear::Model misplaced_model_desired_coords) {
    osrf_gear::Model model;
    vector<osrf_gear::Model> desired, satisfied_models_wrt_world, missing_models_wrt_world, orphan_models_wrt_world, misplaced_models_actual_coords, misplaced_models_desired_coords;
    //spart_to_model(part,model);
    model.type = part.name;
    model.pose = part.pose.pose;
    desired.clear();
    desired.push_back(model);
    if (!update_inspection(desired, satisfied_models_wrt_world, misplaced_models_actual_coords, misplaced_models_desired_coords, missing_models_wrt_world, orphan_models_wrt_world)) {
        return 0;
    }
    if (misplaced_models_desired_coords.size() == 0) {
        ROS_INFO("pre drop off check good");
        return 0;
    } else {
        misplaced_model_desired_coords = misplaced_models_desired_coords[0];
        misplaced_model_actual_coords = misplaced_models_actual_coords[0];
        return 1;
    }

}

bool BoxInspector::compare_pose(geometry_msgs::Pose pose_A, geometry_msgs::Pose pose_B) {
    Eigen::Affine3d affine1, affine2;
    Eigen::Vector3d origin_diff;
    affine1 = xformUtils_.transformPoseToEigenAffine3d(pose_A);
    affine2 = xformUtils_.transformPoseToEigenAffine3d(pose_B);
    origin_diff = affine1.translation() - affine2.translation();
    double origin_err = origin_diff.norm();
    Eigen::Matrix3d R1, R2, R_diff;
    R1 = affine1.linear();
    R2 = affine2.linear();
    R_diff = R1.inverse() * R2;
    Eigen::AngleAxisd angleAxis(R_diff);
    double rotation_err = angleAxis.angle();
    if (origin_err < 0.01 && rotation_err < 0.1) {
        return true;
    } else {
        return false;
    }
}

bool BoxInspector::compare_pose(geometry_msgs::PoseStamped pose_stamped_A, geometry_msgs::PoseStamped pose_stamped_B) {
    geometry_msgs::Pose pose_A, pose_B;
    pose_A = pose_stamped_A.pose;
    pose_B = pose_stamped_B.pose;
    bool comparison = compare_pose(pose_A, pose_B);
    return comparison;
}

bool BoxInspector::compare_pose_approx(geometry_msgs::Pose pose_A, geometry_msgs::Pose pose_B) {
    Eigen::Affine3d affine1, affine2;
    Eigen::Vector3d origin_diff;
    affine1 = xformUtils_.transformPoseToEigenAffine3d(pose_A);
    affine2 = xformUtils_.transformPoseToEigenAffine3d(pose_B);
    origin_diff = affine1.translation() - affine2.translation();
    double origin_err = origin_diff.norm();
    Eigen::Matrix3d R1, R2, R_diff;
    R1 = affine1.linear();
    R2 = affine2.linear();
    R_diff = R1.inverse() * R2;
    Eigen::AngleAxisd angleAxis(R_diff);
    double rotation_err = angleAxis.angle();
    if (origin_err < 0.03 && rotation_err < 0.3) {
        return true;
    } else {
        return false;
    }
}

bool BoxInspector::compare_pose_approx(geometry_msgs::PoseStamped pose_stamped_A, geometry_msgs::PoseStamped pose_stamped_B) {
    geometry_msgs::Pose pose_A, pose_B;
    pose_A = pose_stamped_A.pose;
    pose_B = pose_stamped_B.pose;
    bool comparison = compare_pose_approx(pose_A, pose_B);
    return comparison;
}


//to request a new snapshot, set need_new_snapshot_ = true, and make sure to give a ros::spinOnce()

void BoxInspector::box_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    if (!got_new_snapshot_) {
        box_inspector_image_ = *image_msg; //copy the current message to a member data var, i.e. freeze the snapshot
        got_new_snapshot_ = true;
        //ROS_INFO_STREAM("received box-camera image of: " << box_inspector_image_ << endl);
        int n_models = box_inspector_image_.models.size();
        //ROS_INFO("%d models seen ", n_models);
    }
}

//method to request a new snapshot from logical camera; blocks until snapshot is ready,
// then result will be in box_inspector_image_

bool BoxInspector::get_new_snapshot_from_box_cam() {
    got_new_snapshot_ = false;
    double dt = 0.05;
    double timer = 0.0;
    while (!got_new_snapshot_ && (timer < BOX_INSPECTOR_TIMEOUT)) {
        ros::spinOnce();
        ros::Duration(dt).sleep();
        timer += dt;
    }
    if (timer >= BOX_INSPECTOR_TIMEOUT) {
        ROS_WARN("could not update box inspection image!");
        return false;
    } else {
        return true;
    }
}

//averages multiple snapshots; returns a LogicalCameraImage with coordinates wrt camera frame
bool BoxInspector::get_filtered_snapshots_from_box_cam(osrf_gear::LogicalCameraImage &filtered_box_camera_image) {
    int n_snapshots = 3; //choose to average this many snapshots
    vector<geometry_msgs::Pose> sum_poses, averaged_poses;
    //osrf_gear::LogicalCameraImage filtered_box_camera_image;

    if (!get_new_snapshot_from_box_cam()) {
        return false;
    } //failed to get new image; blackout?

    //got at least one snapshot; proceed to average
    int num_parts_seen = box_inspector_image_.models.size();
    averaged_poses.resize(num_parts_seen);
    sum_poses.resize(num_parts_seen);

    //filtered_box_camera_image.models.resize(num_parts_seen);
    //ROS_INFO("update_inspection: box camera saw %d objects", num_parts_seen);

    for (int i = 0; i < num_parts_seen; i++) { //initialize, based on first successful snapshot
        sum_poses[i] = box_inspector_image_.models[i].pose;
    }

    int i_snapshots = 1; //count how many good snapshots are to be included in  average
    for (int i = 0; i < n_snapshots; i++) { // try for this many more snapshots
        if (get_new_snapshot_from_box_cam()) {
            //filtered_box_camera_image = box_inspector_image_;
            if (box_inspector_image_.models.size() == num_parts_seen) { //if here, got a new snapshot consistent w/ first snapshot;
                //start  averaging process
                i_snapshots++;
                for (int j = 0; j < num_parts_seen; j++) {
                    sum_poses[j].position.x += box_inspector_image_.models[j].pose.position.x;
                    sum_poses[j].position.y += box_inspector_image_.models[j].pose.position.y;
                    sum_poses[j].position.z += box_inspector_image_.models[j].pose.position.z;
                    sum_poses[j].orientation.x += box_inspector_image_.models[j].pose.orientation.x;
                    sum_poses[j].orientation.y += box_inspector_image_.models[j].pose.orientation.y;
                    sum_poses[j].orientation.z += box_inspector_image_.models[j].pose.orientation.z;
                    sum_poses[j].orientation.w += box_inspector_image_.models[j].pose.orientation.w;

                }
            }
        }
    }
    //compute averages:
    for (int j = 0; j < num_parts_seen; j++) {
        averaged_poses[j].position.x = sum_poses[j].position.x / i_snapshots;
        averaged_poses[j].position.y = sum_poses[j].position.y / i_snapshots;
        averaged_poses[j].position.z = sum_poses[j].position.z / i_snapshots;
        averaged_poses[j].orientation.x = sum_poses[j].orientation.x / i_snapshots;
        averaged_poses[j].orientation.y = sum_poses[j].orientation.y / i_snapshots;
        averaged_poses[j].orientation.z = sum_poses[j].orientation.z / i_snapshots;
        averaged_poses[j].orientation.w = sum_poses[j].orientation.w / i_snapshots;
        //normalize the averaged  quaternion:
        double quat_norm = sqrt(pow(averaged_poses[j].orientation.x, 2.0) + pow(averaged_poses[j].orientation.y, 2.0) + pow(averaged_poses[j].orientation.z, 2.0) + pow(averaged_poses[j].orientation.w, 2.0)); //SHOULD AVOID PERFORMANCE OVERHEAD OF MULTIPLE FNC CALL; change to x*x instead
        averaged_poses[j].orientation.x /= quat_norm;
        averaged_poses[j].orientation.y /= quat_norm;
        averaged_poses[j].orientation.z /= quat_norm;
        averaged_poses[j].orientation.w /= quat_norm;

    }

    //put these poses into a logical-camera image message:
    filtered_box_camera_image = box_inspector_image_; //this sets dimension, part  names,  and camera pose
    //NOTE: all  coords are w/rt box camera frame
    for (int i = 0; i < num_parts_seen; i++) {
        filtered_box_camera_image.models[i].pose = averaged_poses[i];
    }
    return true;
}



//here is  the main fnc; provide a list of models, expressed as desired parts w/ poses w/rt box;
//get a box-camera logical image and  parse it
//populate the vectors as follows:
// satisfied_models_wrt_world: vector of models that match shipment specs, including precision location in box
// misplaced_models_wrt_world: vector of models that belong in the shipment, but are imprecisely located in box
// missing_models_wrt_world:  vector of models that are requested in the shipment, but not yet present in the box
// orphan_models_wrt_world: vector of models that are seen in the box, but DO NOT belong in the box

bool BoxInspector::update_inspection(vector<osrf_gear::Model> desired_models_wrt_world,
        vector<osrf_gear::Model> &satisfied_models_wrt_world,
        vector<osrf_gear::Model> &misplaced_models_actual_coords_wrt_world,
        vector<osrf_gear::Model> &misplaced_models_desired_coords_wrt_world,
        vector<osrf_gear::Model> &missing_models_wrt_world,
        vector<osrf_gear::Model> &orphan_models_wrt_world) {
    int ans; // FOR DEBUG
    osrf_gear::LogicalCameraImage filtered_box_camera_image;

    if (!get_filtered_snapshots_from_box_cam(filtered_box_camera_image)) {
        return false;
    }

    /*
    int n_snapshots = 3;
    got_new_snapshot_ = false;
    vector<geometry_msgs::Pose> sum_pose, average_pose;

    filtered_box_camera_image = box_inspector_image_;
    if (!get_new_snapshot_from_box_cam()) {
        return false;
    } //failed to get new image; blackout?

    //got at least one snapshot; proceed to average
    int num_parts_seen = box_inspector_image_.models.size();
    ROS_INFO("update_inspection: box camera saw %d objects", num_parts_seen);
    sum_pose.resize(num_parts_seen);
    average_pose.resize(num_parts_seen);
    for (int i = 0; i < num_parts_seen; i++) {
        sum_pose[i].position.x = 0;
        sum_pose[i].position.y = 0;
        sum_pose[i].position.z = 0;
        sum_pose[i].orientation.x = 0;
        sum_pose[i].orientation.y = 0;
        sum_pose[i].orientation.z = 0;
        sum_pose[i].orientation.w = 0;

    }
    int i_snapshots = 0;
    for (int i = 0; i < n_snapshots; i++) { // Should remove hard coded numbers and use while
        if (get_new_snapshot_from_box_cam()) {
            filtered_box_camera_image = box_inspector_image_;
            if (box_inspector_image_.models.size() == num_parts_seen) {
                i_snapshots++;
                for (int j = 0; j < num_parts_seen; j++) {
                    sum_pose[j].position.x += box_inspector_image_.models[j].pose.position.x;
                    sum_pose[j].position.y += box_inspector_image_.models[j].pose.position.y;
                    sum_pose[j].position.z += box_inspector_image_.models[j].pose.position.z;
                    sum_pose[j].orientation.x += box_inspector_image_.models[j].pose.orientation.x;
                    sum_pose[j].orientation.y += box_inspector_image_.models[j].pose.orientation.y;
                    sum_pose[j].orientation.z += box_inspector_image_.models[j].pose.orientation.z;
                    sum_pose[j].orientation.w += box_inspector_image_.models[j].pose.orientation.w;

                }
            }
        }
    }
    if (i_snapshots > 0) {
        for (int j = 0; j < num_parts_seen; j++) {
            average_pose[j].position.x = sum_pose[j].position.x / i_snapshots;
            average_pose[j].position.y = sum_pose[j].position.y / i_snapshots;
            average_pose[j].position.z = sum_pose[j].position.z / i_snapshots;
            average_pose[j].orientation.x = sum_pose[j].orientation.x / i_snapshots;
            average_pose[j].orientation.y = sum_pose[j].orientation.y / i_snapshots;
            average_pose[j].orientation.z = sum_pose[j].orientation.z / i_snapshots;
            average_pose[j].orientation.w = sum_pose[j].orientation.w / i_snapshots;
            double n = sqrt(pow(average_pose[j].orientation.x, 2.0) + pow(average_pose[j].orientation.y, 2.0) + pow(average_pose[j].orientation.z, 2.0) + pow(average_pose[j].orientation.w, 2.0)); //SHOULD AVOID PERFORMANCE OVERHEAD OF MULTIPLE FNC CALL; change to x*x instead
            average_pose[j].orientation.x /= n;
            average_pose[j].orientation.y /= n;
            average_pose[j].orientation.z /= n;
            average_pose[j].orientation.w /= n;

        }


        for (int i = 0; i < num_parts_seen; i++) {
            filtered_box_camera_image.models[i].pose = average_pose[i];
        }
     */

    //ROS_INFO_STREAM("filtered box camera image" << filtered_box_camera_image);
    int num_parts_seen = filtered_box_camera_image.models.size();
    int num_parts_desired = desired_models_wrt_world.size();
    orphan_models_wrt_world.clear(); //this will be empty, unless something very odd happens
    satisfied_models_wrt_world.clear(); //shipment will be complete when this matches parts/poses specified in shipment
    misplaced_models_actual_coords_wrt_world.clear();
    misplaced_models_desired_coords_wrt_world.clear();
    missing_models_wrt_world.clear();
    int num_each_parts_seen[5] = {0, 0, 0, 0, 0};
    int num_each_parts_des[5] = {0, 0, 0, 0, 0};
    for (int i = 0; i < num_parts_seen; i++) {
        switch (name_to_part_id_mappings[filtered_box_camera_image.models[i].type]) {
            case 1:
                num_each_parts_seen[0] += 1;
                break;
            case 2:
                num_each_parts_seen[1] += 1;
                break;
            case 3:
                num_each_parts_seen[2] += 1;
                break;
            case 4:
                num_each_parts_seen[3] += 1;
                break;
            case 5:
                num_each_parts_seen[4] += 1;
                break;
        }
    }
    for (int i = 0; i < num_parts_desired; i++) {
        switch (name_to_part_id_mappings[desired_models_wrt_world[i].type]) {
            case 1:
                num_each_parts_des[0] += 1;
                break;
            case 2:
                num_each_parts_des[1] += 1;
                break;
            case 3:
                num_each_parts_des[2] += 1;
                break;
            case 4:
                num_each_parts_des[3] += 1;
                break;
            case 5:
                num_each_parts_des[4] += 1;
                break;
        }

    }

    for (int i = 0; i < filtered_box_camera_image.models.size(); i++) {
        if (filtered_box_camera_image.models[i].type != "shipping_box") {
            bool found = false;
            for (int j = 0; j < desired_models_wrt_world.size(); j++) {
                if (filtered_box_camera_image.models[i].type == desired_models_wrt_world[j].type) {
                    geometry_msgs::PoseStamped model_pose_from_image_wrt_world = compute_stPose(filtered_box_camera_image.pose, filtered_box_camera_image.models[i].pose);
                    bool pose_comparison = compare_pose(model_pose_from_image_wrt_world.pose, desired_models_wrt_world[j].pose);
                    if (pose_comparison) {
                        found = true;
                    }
                }
            }
            if (found) {
                geometry_msgs::PoseStamped pose_wrt_world = compute_stPose(filtered_box_camera_image.pose, filtered_box_camera_image.models[i].pose);
                osrf_gear::Model model;
                model = filtered_box_camera_image.models[i];
                model.pose = pose_wrt_world.pose;
                satisfied_models_wrt_world.push_back(model);
            }
        }
    }

    for (int ipart = 0; ipart < 5; ipart++) {


        if (num_each_parts_seen[ipart] < num_each_parts_des[ipart]) {
            for (int j = 0; j < desired_models_wrt_world.size(); j++) {
                if (desired_models_wrt_world[j].type == part_id_to_name_mappings[ipart + 1]) {
                    bool found = false;
                    for (int i = 0; i < filtered_box_camera_image.models.size(); i++) {
                        if (filtered_box_camera_image.models[i].type == desired_models_wrt_world[j].type) {
                            geometry_msgs::PoseStamped model_pose_from_image_wrt_world = compute_stPose(filtered_box_camera_image.pose, filtered_box_camera_image.models[i].pose);
                            bool pose_comparison = compare_pose(model_pose_from_image_wrt_world.pose, desired_models_wrt_world[j].pose);
                            if (pose_comparison) {
                                found = true;
                            }
                        }
                    }
                    if (!found) {
                        //ROS_INFO("found missing");
                        missing_models_wrt_world.push_back(desired_models_wrt_world[j]);
                    }
                }
            }

        } else if (num_each_parts_seen[ipart] > num_each_parts_des[ipart]) {
            for (int i = 0; i < filtered_box_camera_image.models.size(); i++) {

                if (filtered_box_camera_image.models[i].type == part_id_to_name_mappings[ipart + 1]) {


                    bool found = false;
                    for (int j = 0; j < desired_models_wrt_world.size(); j++) {
                        if (filtered_box_camera_image.models[i].type == desired_models_wrt_world[j].type) {
                            geometry_msgs::PoseStamped model_pose_from_image_wrt_world = compute_stPose(filtered_box_camera_image.pose, filtered_box_camera_image.models[i].pose);
                            bool pose_comparison = compare_pose(model_pose_from_image_wrt_world.pose, desired_models_wrt_world[j].pose);
                            if (pose_comparison) {
                                found = true;
                            }
                        }
                    }
                    if (!found) {
                        geometry_msgs::PoseStamped pose_wrt_world = compute_stPose(filtered_box_camera_image.pose, filtered_box_camera_image.models[i].pose);
                        osrf_gear::Model model;
                        model = filtered_box_camera_image.models[i];
                        model.pose = pose_wrt_world.pose;
                        orphan_models_wrt_world.push_back(model);
                    }

                }
            }
        } else if (num_each_parts_des[ipart] == num_each_parts_seen[ipart]) {
            for (int i = 0; i < filtered_box_camera_image.models.size(); i++) {
                if (filtered_box_camera_image.models[i].type == part_id_to_name_mappings[ipart + 1]) {
                    bool found = false;
                    for (int j = 0; j < desired_models_wrt_world.size(); j++) {
                        if (filtered_box_camera_image.models[i].type == desired_models_wrt_world[j].type) {
                            geometry_msgs::PoseStamped model_pose_from_image_wrt_world = compute_stPose(filtered_box_camera_image.pose, filtered_box_camera_image.models[i].pose);
                            bool pose_comparison = compare_pose(model_pose_from_image_wrt_world.pose, desired_models_wrt_world[j].pose);
                            if (pose_comparison) {
                                found = true;
                            }
                        }
                    }

                    if (!found) {
                        geometry_msgs::PoseStamped pose_wrt_world = compute_stPose(filtered_box_camera_image.pose, filtered_box_camera_image.models[i].pose);
                        osrf_gear::Model model;
                        model = filtered_box_camera_image.models[i];
                        model.pose = pose_wrt_world.pose;
                        misplaced_models_actual_coords_wrt_world.push_back(model);
                    }


                }
            }


            for (int j = 0; j < desired_models_wrt_world.size(); j++) {
                if (desired_models_wrt_world[j].type == part_id_to_name_mappings[ipart + 1]) {
                    bool found = false;
                    for (int i = 0; i < filtered_box_camera_image.models.size(); i++) {
                        if (filtered_box_camera_image.models[i].type == desired_models_wrt_world[j].type) {
                            geometry_msgs::PoseStamped model_pose_from_image_wrt_world = compute_stPose(filtered_box_camera_image.pose, filtered_box_camera_image.models[i].pose);
                            bool pose_comparison = compare_pose(model_pose_from_image_wrt_world.pose, desired_models_wrt_world[j].pose);

                            if (pose_comparison) {
                                found = true;
                            }
                        }
                    }

                    if (!found) {

                        misplaced_models_desired_coords_wrt_world.push_back(desired_models_wrt_world[j]);
                    }
                }
            }


        }

    }
    
    //issue here: a bad part can show up as both an "orphan" and a desired part
    //also, from quality sensor, do not get part name; need to find part name to
    // compute offset for part removal
    // further, there may be more than 1 faulty part, but only the first found faulty part
    // is listed
    //probably should create a separate vector for faulty parts vs "orphan"  parts
    ros::spinOnce();
    if (qual_sensor_1_sees_faulty_part_) {
        osrf_gear::Model bad_model;
        part_to_model(bad_part_Qsensor1_, bad_model);
        orphan_models_wrt_world.push_back(bad_model);

    }

    if (qual_sensor_2_sees_faulty_part_) {
        osrf_gear::Model bad_model;
        part_to_model(bad_part_Qsensor2_, bad_model);
        orphan_models_wrt_world.push_back(bad_model);
    }
    return 1;

}

//this function really only works for boxcam 1;
//at Q2, expect to ONLY look for bad parts per inspection cam 2

//as above, but with more args for org by part indices
bool BoxInspector::update_inspection(vector<osrf_gear::Model> desired_models_wrt_world,
        vector<osrf_gear::Model> &satisfied_models_wrt_world,
        vector<osrf_gear::Model> &misplaced_models_actual_coords_wrt_world,
        vector<osrf_gear::Model> &misplaced_models_desired_coords_wrt_world,
        vector<osrf_gear::Model> &missing_models_wrt_world,
        vector<osrf_gear::Model> &orphan_models_wrt_world,
        vector<int> &part_indices_missing,
        vector<int> &part_indices_misplaced,
        vector<int> &part_indices_precisely_placed) {
    int ans; // FOR DEBUG
    osrf_gear::LogicalCameraImage filtered_box_camera_image;
    osrf_gear::Model test_model, desired_model;
    geometry_msgs::PoseStamped model_pose_from_image_wrt_world;

    //if blackout, DO NOT clear the model vectors!
    if (!get_filtered_snapshots_from_box_cam(filtered_box_camera_image)) {
        return false;
    }

    //OK--got an image; can clear out and rebuild all model vectors
    orphan_models_wrt_world.clear(); //this will be empty, unless something very odd happens
    satisfied_models_wrt_world.clear(); //shipment will be complete when this matches parts/poses specified in shipment
    misplaced_models_actual_coords_wrt_world.clear();
    misplaced_models_desired_coords_wrt_world.clear();
    missing_models_wrt_world.clear();
    part_indices_misplaced.clear();
    part_indices_missing.clear();
    part_indices_precisely_placed.clear();

    //ROS_INFO_STREAM("filtered box camera image" << filtered_box_camera_image);
    int num_parts_seen = filtered_box_camera_image.models.size();
    int num_parts_desired = desired_models_wrt_world.size();
    vector<bool> classified_observed_part, classified_desired_part;
    //use this to keep track of which observed  parts have been classified
    for (int i = 0; i < num_parts_seen; i++) classified_observed_part.push_back(false);
    for (int i = 0; i < num_parts_desired; i++) classified_desired_part.push_back(false);

    //don't consider the shipping box as a part:
    string box_name("shipping_box");
       for (int ipart_seen = 0; ipart_seen < num_parts_seen; ipart_seen++) {
            test_model = filtered_box_camera_image.models[ipart_seen];
            string test_model_name(test_model.type);
            if (box_name == test_model_name) classified_observed_part[ipart_seen] = true;
       }
    
    
    //start with testing for bad parts:
    inventory_msgs::Part bad_part;
    string model_name;
    if (get_bad_part_Q1(bad_part)) {
        //found a bad part; match it to filtered image and classify it as orphaned
        bool found = false;
        int ipart_seen = 0;
        string bad_part_name(bad_part.name);


        //start search through seen images:
        while ((!found)&&(ipart_seen < num_parts_seen)) {
            test_model = filtered_box_camera_image.models[ipart_seen];
            //string test_model_name(test_model.type);
            //if (test_model_name != bad_part_name) {
                //no match here; try next model seen
            //    ipart_seen++;
            //} else { //name matches; do coordinates match, image to bad_part?
                //convert observed part pose to world coords:
                model_pose_from_image_wrt_world = compute_stPose(filtered_box_camera_image.pose, test_model.pose);
                //bad_part pose is already in world coords
                if (compare_pose(model_pose_from_image_wrt_world.pose, bad_part.pose.pose)) {
                    //found match!  record it
                    found = true;
                    classified_observed_part[ipart_seen] = true;
                    test_model.pose = model_pose_from_image_wrt_world.pose;
                    orphan_models_wrt_world.push_back(test_model);
                    ROS_WARN("found a bad part--classified as orphaned");
                } else {
                    ipart_seen++; //not a good location match; try next candidate
                }
           // }
        }

        if (!found) {
            ROS_WARN("update_inspection: SOMETHING IS WRONG.  bad part reported, but does not match any parts observed by logical cam ");
            ROS_WARN("there seems to be a logical error in this code");
        }
    } else {
        ROS_INFO("no bad parts reported by quality sensor 1");
    }
    //presumably, when reach here, if any bad parts have been seen, the first one is classified as orphaned

    //next, look for precise matches:
    //ROS_INFO("seeking precise matches");
    for (int ipart_seen = 0; ipart_seen < num_parts_seen; ipart_seen++) {
        //only consider observed parts not already classified
        if (!classified_observed_part[ipart_seen]) {
            test_model = filtered_box_camera_image.models[ipart_seen];
            string test_model_name(test_model.type);
            model_pose_from_image_wrt_world = compute_stPose(filtered_box_camera_image.pose, test_model.pose);
            int i_des_part = 0;
            bool found = false;
            while ((i_des_part < num_parts_desired)&&(!found)) {
                //step through all desired parts seeking a good match:
                desired_model = desired_models_wrt_world[i_des_part];
                string desired_model_name(desired_model.type);
                if (desired_model_name != test_model_name) {
                    //this  can't be it--names  don't match
                    //ROS_INFO("names don't match for i_des_part = %d, ipart_seen = %d", i_des_part, ipart_seen);
                    i_des_part++;
                } else {
                    //OK--names match; how about coordinates?
                    if (compare_pose(model_pose_from_image_wrt_world.pose, desired_model.pose)) {
                        //found a match!
                        //ROS_INFO("found match! i_des_part = %d, ipart_seen = %d", i_des_part, ipart_seen);
                        found = true; //done considering precise match for this observed part; move along to next observed  part
                        classified_observed_part[ipart_seen] = true;
                        classified_desired_part[i_des_part] = true;
                        satisfied_models_wrt_world.push_back(desired_model);
                        part_indices_precisely_placed.push_back(i_des_part);
                    } else {
                        //nope--not a match; try next desired  part
                        //ROS_INFO("names matched, but not coords for i_des_part = %d, ipart_seen = %d", i_des_part, ipart_seen);
                        i_des_part++;
                    }
                }
            }
        }
    }
    //ROS_INFO("found %d precise matches", (int) satisfied_models_wrt_world.size());

    //remaining parts  seen are either misaligned or orphaned
    //ROS_INFO("seeking approximate matches");
    for (int ipart_seen = 0; ipart_seen < num_parts_seen; ipart_seen++) {
        //only consider observed parts not already classified
        if (!classified_observed_part[ipart_seen]) {
            //ROS_INFO("evaluating ipart_seen = %d", ipart_seen);
            test_model = filtered_box_camera_image.models[ipart_seen];
            string test_model_name(test_model.type);
            model_pose_from_image_wrt_world = compute_stPose(filtered_box_camera_image.pose, test_model.pose);
            int i_des_part = 0;
            bool found = false;
            while ((i_des_part < num_parts_desired)&&(!found)) {
                if (!classified_desired_part[i_des_part]) {
                    //step through all desired parts seeking a good match:
                    desired_model = desired_models_wrt_world[i_des_part];
                    string desired_model_name(desired_model.type);
                    if (desired_model_name != test_model_name) {
                        //this  can't be it--names  don't match
                        i_des_part++;
                    } else {
                        //OK--names match; how about coordinates?
                        if (compare_pose_approx(model_pose_from_image_wrt_world.pose, desired_model.pose)) {
                            //ROS_INFO("found approx match for ipart_seen= %d, i_des_part= %d", ipart_seen, i_des_part);
                            //found a match!
                            found = true; //done considering precise match for this observed part; move along to next observed  part
                            classified_observed_part[ipart_seen] = true;
                            classified_desired_part[i_des_part] = true;
                            misplaced_models_desired_coords_wrt_world.push_back(desired_model);
                            test_model.pose = model_pose_from_image_wrt_world.pose;
                            misplaced_models_actual_coords_wrt_world.push_back(test_model);
                            part_indices_misplaced.push_back(i_des_part);
                        } else {
                            //nope--not a match; try next desired  part
                            i_des_part++;
                        }
                    }
                }
                else  i_des_part++;
            }
        }
        
    }
    //ROS_INFO("found %d approximate matches", (int) misplaced_models_actual_coords_wrt_world.size());

    //are there any stragglers?  If so, they are badly placed, or they are orphans
    //ROS_INFO("checking for outliers");
    for (int ipart_seen = 0; ipart_seen < num_parts_seen; ipart_seen++) {
        //only consider observed parts not already classified
        if (!classified_observed_part[ipart_seen]) {
            test_model = filtered_box_camera_image.models[ipart_seen];
            model_pose_from_image_wrt_world = compute_stPose(filtered_box_camera_image.pose, test_model.pose);

            string test_model_name(test_model.type);
            int i_des_part = 0;
            bool found = false;
            while ((i_des_part < num_parts_desired)&&(!found)) {
                if (!classified_desired_part[i_des_part]) {
                    //step through all desired parts seeking a good match:
                    desired_model = desired_models_wrt_world[i_des_part];
                    string desired_model_name(desired_model.type);
                    if (desired_model_name == test_model_name) {
                        //matched names of unclassified  parts; associate them
                        found = true;
                        classified_observed_part[ipart_seen] = true;
                        classified_desired_part[i_des_part] = true;
                        misplaced_models_desired_coords_wrt_world.push_back(desired_model);
                        test_model.pose = model_pose_from_image_wrt_world.pose;
                        misplaced_models_actual_coords_wrt_world.push_back(test_model);
                        part_indices_misplaced.push_back(i_des_part);
                    } else {
                        i_des_part++;
                    }

                }
                else i_des_part++;
            }
            //finished match search for this observed part; if no match, call  it an orphan
            if (i_des_part >= num_parts_desired) {
                //halting condition was end of list of desired  parts; therefore, this observed part is an orphan
                classified_observed_part[ipart_seen] = true;
                test_model.pose = model_pose_from_image_wrt_world.pose;
                orphan_models_wrt_world.push_back(test_model);
            }
        }
    }
    
    //by the time we are here, all parts should be classified
    bool all_classified = true;
    for (int i = 0; i < num_parts_seen; i++) all_classified &= classified_observed_part[i];
    if (!all_classified) {
        ROS_WARN("update_inspection: something is wrong; not all observed  parts are classified");
        return false;
    }
    
    //now, all unclassified desired parts are missing:
    for (int ipart=0;ipart<num_parts_desired;ipart++) {
        if (!classified_desired_part[ipart]) {
            //this part is not  classified,  so it's missing:
            part_indices_missing.push_back(ipart);
            desired_model = desired_models_wrt_world[ipart];
            missing_models_wrt_world.push_back(desired_model);
        }
    }
    
    
    int n_orphaned = orphan_models_wrt_world.size();
    int n_satisifed = satisfied_models_wrt_world.size();
    int n_misplaced = misplaced_models_actual_coords_wrt_world.size();
    //ROS_INFO("box inspection update results: ");
    //ROS_INFO("num_parts_seen= %d; n_satisifed=%d; n_misplaced=%d; n_orphaned = %d", num_parts_seen, n_satisifed, n_misplaced, n_orphaned);
    return true;

}

//intent of this function: when holding a part above the (Q1) box, find the (filtered) pose of
// the part with respect to world coords; this is used to identify the actual grasp transform
// return false if camera not working or not credible interpretation
//   observed_part.pose = grasped_pose_wrt_wrld;

bool BoxInspector::get_grasped_part_pose_wrt_world(inventory_msgs::Part &observed_part) {
    //get a new (filtered) snapshot of the box-inspection camera:
    osrf_gear::LogicalCameraImage filtered_box_camera_image;
    geometry_msgs::PoseStamped grasped_part_pose_wrt_world;
    string grasped_part_name(observed_part.name); 
    ROS_INFO_STREAM("looking for grasped part name "<<grasped_part_name<<endl);    
    if (!get_filtered_snapshots_from_box_cam(filtered_box_camera_image)) {
        ROS_WARN("could not observe grasp--could not get image");
        return false;
    }
    
    //convert each part to world coords, and find the part with max z value;
    int num_models = filtered_box_camera_image.models.size();
    //int highest_model = 0;    
    if (num_models < 2) {
        ROS_WARN("grasped_part_pose sensing: only 1 model seen; giving up");
        return false; // must see box and at least one more model!
    }
    //next, make sure at least one of the models seen matches the intended part name:
    double max_ht = 0.0; //BOX_SURFACE_HT_WRT_WORLD;
    
    bool found_a_candidate=false;        
    for (int i = 0; i < box_inspector_image_.models.size(); i++) {
        string model_name(box_inspector_image_.models[i].type);
        if (model_name==grasped_part_name) {
            found_a_candidate=true;
            grasped_part_pose_wrt_world = compute_stPose(box_inspector_image_.pose, box_inspector_image_.models[i].pose);
            if (grasped_part_pose_wrt_world.pose.position.z > max_ht) {
                max_ht = grasped_part_pose_wrt_world.pose.position.z;
                //winner = i; //don't care which model wins; just copy over the pose
                observed_part.pose = grasped_part_pose_wrt_world;
            }
        }
    }
    if (!found_a_candidate) {
        ROS_WARN("grasped_part_pose sensing: could not match name of part to any observed parts; giving up");
        return false; // certainly did not see intended grasped part
    }
    ROS_INFO_STREAM("presumed grasped_part_pose_wrt_world = "<<endl<<grasped_part_pose_wrt_world<<endl);
    return true;
}



//intent of this function is, get a snapshot from the box-inspection camera;
//parse the image data to see if a shipping box is present
//if not, return false
//if seen, transform box pose to box pose w/rt world frame,
//    copy data to reference arg box_pose_wrt_world
//    and return "true"

bool BoxInspector::get_box_pose_wrt_world(geometry_msgs::PoseStamped &box_pose_wrt_world) {
    geometry_msgs::Pose cam_pose, box_pose; //cam_pose is w/rt world, but box_pose is w/rt camera
    //default: assign assumed box pose until/unless get camera data
    box_pose_wrt_world = NOM_BOX1_POSE_WRT_WORLD; //did not see box; use expected pose

    //get a new (filtered) snapshot of the box-inspection camera:
    osrf_gear::LogicalCameraImage filtered_box_camera_image;
    if (!get_filtered_snapshots_from_box_cam(filtered_box_camera_image)) {
        return false;
    }


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

            box_pose_wrt_world = compute_stPose(cam_pose, box_pose);
            ROS_INFO_STREAM("box_pose_wrt_world: " << box_pose_wrt_world << endl);
            return true;
        }
    }
    //if reach here, did not find shipping_box
    ROS_WARN("get_box_pose_wrt_world(): shipping-box not seen!");

    return false;
}

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

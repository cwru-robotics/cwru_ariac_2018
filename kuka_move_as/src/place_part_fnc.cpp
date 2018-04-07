//use this fnc to place a specified, grasped part in a box, either at inspection station 1 or 2
// upon completion of "place" server will return either:
//  NO_ERROR, or:  UNREACHABLE, PART_DROPPED, WRONG_PARAMETER, or GRIPPER_FAULT
//


unsigned short int KukaBehaviorActionServer::place_part_in_box_no_release(inventory_msgs::Part part) {
    unsigned short int errorCode = kuka_move_as::RobotBehaviorResult::NO_ERROR; //return this if ultimately successful
    trajectory_msgs::JointTrajectory transition_traj;
    //inventory_msgs::Part part = goal->destinationPart;
    ROS_INFO("The part is %s; it should be placed in  %s ", part.name.c_str(),
            placeFinder_[part.location].c_str());
    ROS_INFO("part info: ");
    ROS_INFO_STREAM(part);
    errorCode_ = compute_box_dropoff_key_poses(part);
    if (errorCode_ != kuka_move_as::RobotBehaviorResult::NO_ERROR) {
        return errorCode_;
    }
    //extract box location codes from Part:
    int current_hover_code = location_to_pose_code_map[part.location];
    int current_cruise_code = location_to_cruise_code_map[part.location];
    
    if (!move_posecode1_to_posecode2(current_pose_code_, current_hover_code)) {

        ROS_WARN("error with move between pose codes");
        errorCode_ = kuka_move_as::RobotBehaviorResult::PRECOMPUTED_TRAJ_ERR; //inform our client of error code
        return errorCode_;
    }
    is_attached_ = gripperInterface_.isGripperAttached();
    if (!is_attached_) {
            errorCode_ = kuka_move_as::RobotBehaviorResult::PART_DROPPED;
            return errorCode_;
    }

    ROS_WARN(" DO DROPOFF STEPS HERE...");


    //now move to approach_dropoff_jspace_pose_:
    ROS_INFO("moving to approach_dropoff_jspace_pose_ ");
    move_to_jspace_pose(approach_dropoff_jspace_pose_, 1.0); //try it this way instead
   
    //this would be a good place to recompute dropoff pose;
    //BUT need to know actual pose of part, as observed by camera
//bool KukaBehaviorActionServer::recompute_pickup_dropoff_IK(Eigen::Affine3d actual_grasped_part_pose_wrt_world,Eigen::Affine3d desired_part_pose_wrt_world,
//       Eigen::VectorXd &q_vec_soln)    

    ROS_INFO("at approach_dropoff_jspace_pose_; to proceed to dropoff  pose, enter 1: ");
    //cout<<"enter 1: ";
    cin>>ans;
    //now move to desired_grasp_dropoff_pose_:
    ROS_INFO_STREAM("moving to desired_grasp_dropoff_pose_ " << std::endl << desired_grasp_dropoff_pose_.transpose());
    move_to_jspace_pose(desired_grasp_dropoff_pose_, 1.0); //try it this way instead
        
    //check if part is still attached
    is_attached_ = gripperInterface_.isGripperAttached();
    if (!is_attached_) {
        ROS_WARN("dropped part!");
        errorCode = kuka_move_as::RobotBehaviorResult::PART_DROPPED; //debug--return error
        return errorCode;
    }
    ROS_INFO("place-part complete--still grasping part");
    cout<<"enter 1: ";
    cin>>ans;
        errorCode_ = kuka_move_as::RobotBehaviorResult::NO_ERROR; //return success
        return errorCode_;
}

//this function assumes the part is already grasped, and it should  be discarded
//the part is also presumably grasped in either box1 or box2
unsigned short int KukaBehaviorActionServer::discard_grasped_part(inventory_msgs::Part part) {

    trajectory_msgs::JointTrajectory transition_traj;
    int current_hover_code = location_to_pose_code_map[part.location];
    int current_cruise_code = location_to_cruise_code_map[part.location]; 
    if (current_hover_code == Q1_HOVER_CODE) {
        move_to_jspace_pose(q1_hover_pose_, 1.0);
        current_pose_code_=current_hover_code;
    }
    else if (current_hover_code == Q2_HOVER_CODE) {
       move_to_jspace_pose(q2_hover_pose_, 1.0);
       current_pose_code_=current_hover_code;
    }
    else {
        ROS_WARN("discard_grasped_part: location code is not a box code");
        errorCode_ = kuka_move_as::RobotBehaviorResult::WRONG_PARAMETER;
        return errorCode_;
    }
    if (!move_posecode1_to_posecode2(current_pose_code_, Q1_DISCARD_CODE)) {
        ROS_WARN("discard_grasped_part:  error with move between pose codes %d and %d ", current_pose_code_, Q1_DISCARD_CODE);
        return errorCode_;
    }

    ROS_WARN("SHOULD DO PART RELEASE HERE");
    //wait max of  10 sec
    errorCode_ = gripperInterface_.release_fnc(10.0); //this version includes testing for release and timeout monitoring, same as below
    if (errorCode_ != kuka_move_as::RobotBehaviorResult::NO_ERROR)  return errorCode_;

    /*
    gripperInterface_.release();
     is_attached_ = gripperInterface_.isGripperAttached();
     double timer=0;
     double dt =0.1;
     while (is_attached_ && (timer<MAX_BEHAVIOR_SERVER_WAIT_TIME)) {
        ROS_WARN("trying to release part");
        timer+=dt;
       gripperInterface_.release();
       is_attached_ = gripperInterface_.isGripperAttached();     
    }
     if (timer>=MAX_BEHAVIOR_SERVER_WAIT_TIME) {
        errorCode_ = kuka_move_as::RobotBehaviorResult::GRIPPER_FAULT; //debug--return error
        return errorCode_;
    }   
     */

    ROS_INFO("from %d to %d ", Q1_DISCARD_CODE, Q1_CRUISE_CODE);
    if (!move_posecode1_to_posecode2(Q1_DISCARD_CODE, Q1_CRUISE_CODE)) {
        ROS_WARN("error with move between pose codes %d and %d ", Q1_DISCARD_CODE, Q1_CRUISE_CODE);
        return errorCode_;
    }


    errorCode_ = kuka_move_as::RobotBehaviorResult::NO_ERROR; //return success
    return errorCode_;
}

unsigned short int KukaBehaviorActionServer::adjust_part_location_no_release(inventory_msgs::Part part_actual, inventory_msgs::Part part_desired) {
    unsigned short int errorCode = kuka_move_as::RobotBehaviorResult::NO_ERROR; //return this if ultimately successful
    trajectory_msgs::JointTrajectory transition_traj;
    errorCode_ = kuka_move_as::RobotBehaviorResult::NO_ERROR;
    
    geometry_msgs::PoseStamped part_current_pose_wrt_world = part_actual.pose;
    geometry_msgs::PoseStamped part_desired_pose_wrt_world = part_desired.pose;
    ROS_INFO_STREAM("Adjusting part location from "<<endl<<part_current_pose_wrt_world<<" to: "<<endl<<part_desired_pose_wrt_world<<endl);
    
    Eigen::Affine3d affine_part_current_wrt_world = xformUtils_.transformPoseToEigenAffine3d(part_current_pose_wrt_world);
    Eigen::Affine3d affine_part_desired_wrt_world = xformUtils_.transformPoseToEigenAffine3d(part_desired_pose_wrt_world);
    //recompute member var: desired_grasp_dropoff_pose_
    if (!recompute_pickup_dropoff_IK(affine_part_current_wrt_world,affine_part_desired_wrt_world,desired_grasp_dropoff_pose_)) {
        ROS_WARN("recompute_pickup_dropoff_IK() error");
        errorCode = kuka_move_as::RobotBehaviorResult::UNREACHABLE; //debug--return error
        return errorCode;
    }

    //now move to approach_dropoff_jspace_pose_:
    ROS_INFO("moving to approach_dropoff_jspace_pose_ "); //same approach as previously computed
    move_to_jspace_pose(approach_dropoff_jspace_pose_, 1.0); //try it this way instead

    //now move to desired_grasp_dropoff_pose_:
    ROS_INFO_STREAM("moving to recomputed desired_grasp_dropoff_pose_ " << std::endl << desired_grasp_dropoff_pose_.transpose());
    move_to_jspace_pose(desired_grasp_dropoff_pose_, 1.0); //try it this way instead
        
    //check if part is still attached
    is_attached_ = gripperInterface_.isGripperAttached();
    if (!is_attached_) {
        ROS_WARN("dropped part!");
        errorCode = kuka_move_as::RobotBehaviorResult::PART_DROPPED; //debug--return error
        return errorCode;
    }
    ROS_INFO("place-part complete--still grasping part");
    //cout<<"enter 1: ";
    //cin>>ans;
        errorCode_ = kuka_move_as::RobotBehaviorResult::NO_ERROR; //return success

        return errorCode_;
}
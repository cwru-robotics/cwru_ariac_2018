//use this fnc to place a grasped part at specified coords, then stop, with part still grasped
//e.g., useful for placing a part under a quality sensor, then quickly discarding, if bad
// ASSUMES part is already grasped
// ASSUMES robot is already in a safe "cruise" or "hover" pose w/rt the part destination
// will return either:
//  NO_ERROR, or:  UNREACHABLE, PART_DROPPED, or WRONG_PARAMETER

//sequence: goes to hover pose, approach pose, dropoff pose, halts with gripper still attached
//will want to use this for Q1 and Q2 stations

// use "goal", but only need to populate the "sourcePart" component
unsigned short int RobotMoveActionServer::place_part_fnc_no_release(inventory_msgs::Part part) {
    unsigned short int errorCode = robot_move_as::RobotMoveResult::NO_ERROR; //return this if ultimately successful
    ROS_INFO("placing a part,  no release");
    ROS_INFO("part info: ");
    ROS_INFO_STREAM(part);
    unsigned short int bin_code = part.location;
    unsigned short int box_placement_location_code = part.box_placement_location_code;
    int ans;
    //identify or compute 4 poses: cruise, hover, approach, placement
    //check if these are valid:
    errorCode = is_placeable(part);
    if (errorCode!=robot_move_as::RobotMoveResult::NO_ERROR) return errorCode;
    
    //if here, have valid poses; move to hover, cruise, approach and placement:
    ROS_INFO("moving to cruise_jspace_pose_ ");
    
    ///xxx THIS CRUISE POSE IS WRONG
    //fix jspace_cruise mappings
    
    move_to_jspace_pose(q_Q1_cruise_pose_, 2.0);
    ros::Duration(2.0).sleep(); //make sure sleep is consistent w/ specified move time; min 1 sec    
    cout<<"enter 1: ";
    cin>>ans;
    
    //check if part is still attached
    if (!robotInterface.isGripperAttached()) {
        ROS_WARN("dropped part!");
        errorCode = robot_move_as::RobotMoveResult::PART_DROPPED; //debug--return error
        return errorCode;
    }    
    
    //if here, have valid poses; move to hover, cruise, approach and placement:
    ROS_INFO("moving to hover_jspace_pose_ ");
    move_to_jspace_pose(q_Q1_hover_pose_, 1.0);
    ros::Duration(1.0).sleep(); //make sure sleep is consistent w/ specified move time; min 1 sec

            
    //    errorCode = robot_move_as::RobotMoveResult::PART_DROPPED; //debug--return error
    //    return errorCode;
    
        //check if part is still attached
    if (!robotInterface.isGripperAttached()) {
        ROS_WARN("dropped part!");
        errorCode = robot_move_as::RobotMoveResult::PART_DROPPED; //debug--return error
        return errorCode;
    }
    
    //now move to dropoff approach pose:
    //ROS_INFO("moving to approach_dropoff_jspace_pose_ ");
    //move_to_jspace_pose(approach_dropoff_jspace_pose_, 1.0);
    //ros::Duration(1.0).sleep();
    //cout<<"enter 1: ";
    //cin>>ans;
    
    //now move to approach pose:
    ROS_INFO("moving to approach ");
        cout<<"enter 1 to move to approach: ";
    cin>>ans;
    move_to_jspace_pose(approach_dropoff_jspace_pose_, 1.0);
    ros::Duration(1.0).sleep(); //make sure sleep is consistent w/ specified move time; min 1 sec
    cout<<"enter 1: ";
    cin>>ans;    
    


    //now move to dropoff pose:
    ROS_INFO_STREAM("moving to dropoff_jspace_pose_ " << std::endl << dropoff_jspace_pose_.transpose());
    cout<<"enter 1 to move to dropoff pose: ";
    cin>>ans;        
    move_to_jspace_pose(dropoff_jspace_pose_, 2.0); // dropoff pose
    ROS_INFO("concluded call to place_part_fnc_no_release()");
    return errorCode;
}

unsigned short int RobotMoveActionServer::is_placeable(inventory_msgs::Part part) {
    
    unsigned short int errorCode = robot_move_as::RobotMoveResult::NO_ERROR; //return this if ultimately successful
    unsigned short int bin_code = part.location;
    unsigned short int box_placement_location_code = part.box_placement_location_code;
    
    //find the cruise pose for this destination:
    if (!cruise_jspace_pose_w_code(bin_code,box_placement_location_code, q_Q1_cruise_pose_)) {
        ROS_WARN("cruise_jspace_pose_w_code() failed for source destination code %d, placement code %d", 
                (int) bin_code, (int) box_placement_location_code);
        errorCode = robot_move_as::RobotMoveResult::WRONG_PARAMETER;
        return errorCode;
    }
    ROS_INFO_STREAM("destination  cruise pose: " << q_Q1_cruise_pose_.transpose());  
    
    //find the hover pose for this bin
    //q_Q1_cruise_pose_,q_Q1_hover_pose_;
    if (!hover_jspace_pose_w_code(bin_code,box_placement_location_code, q_Q1_hover_pose_)) {
        ROS_WARN("hover_jspace_pose() failed for source destination code %d, placement code %d", 
                (int) bin_code, (int) box_placement_location_code);
        errorCode = robot_move_as::RobotMoveResult::WRONG_PARAMETER;
        return errorCode;
    }
    ROS_INFO_STREAM("destination  hover pose: " << q_Q1_hover_pose_.transpose());
    
    //get nominal dropoff pose: set q_manip_nom_;
    //    bool RobotMoveActionServer::set_q_manip_nom_from_destination_part(Part part) {
    if(!set_q_manip_nom_from_destination_part(part)) {
        ROS_WARN("set_q_nom_from_destination_part() failed for destination code %d, placement code %d", 
                (int) part.location, (int) part.box_placement_location_code);
        errorCode = robot_move_as::RobotMoveResult::WRONG_PARAMETER;
        return errorCode;        
    }
    ROS_INFO_STREAM("nom manipulation pose: "<<q_manip_nom_.transpose()<<endl);

    //need track displacement to get placement pose w/rt base link
    affine_vacuum_pickup_pose_wrt_base_link_ = affine_vacuum_pickup_pose_wrt_base_link(part,
                                                                                       q_Q1_hover_pose_[1]);    
   // xxx THIS IS SCREWED UP!!
//bool RobotMoveActionServer::get_pickup_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,
//                                          Eigen::VectorXd approx_jspace_pose, Eigen::VectorXd &q_vec_soln)    
    ROS_INFO("computing IK for desired manipulation pose: ");
    ROS_INFO_STREAM(affine_vacuum_pickup_pose_wrt_base_link_.translation().transpose());
   if (!get_pickup_IK(affine_vacuum_pickup_pose_wrt_base_link_,  q_manip_nom_, dropoff_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for dropoff pose!");
        errorCode = robot_move_as::RobotMoveResult::UNREACHABLE;
        return errorCode;
    }
    ROS_INFO_STREAM("dropoff_jspace_pose_: " << dropoff_jspace_pose_.transpose());

    ROS_INFO("computing IK for desired manipulation approach pose: ");
    if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, dropoff_jspace_pose_, approach_dist_,
                             approach_dropoff_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for dropoff approach pose!");
        errorCode = robot_move_as::RobotMoveResult::UNREACHABLE;
        return errorCode;
    }
    ROS_INFO_STREAM("approach_dropoff_jspace_pose_: " << approach_dropoff_jspace_pose_.transpose());
    

    return errorCode;
}


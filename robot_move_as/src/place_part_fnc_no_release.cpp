//use this fnc to place a grasped part at specified coords, then stop, with part still grasped
//e.g., useful for placing a part under a quality sensor, then quickly discarding, if bad
// ASSUMES part is already grasped
// also assumes robot is already in a safe "cruise" or "hover" pose w/rt the part destination
// will return either:
//  NO_ERROR, or:  UNREACHABLE, PART_DROPPED, or WRONG_PARAMETER
//
// this fnc returns an error code.  Calling func must provide action-server result

// use "goal", but only need to populate the "sourcePart" component
unsigned short int RobotMoveActionServer::place_part_fnc_no_release(inventory_msgs::Part part) {
    unsigned short int errorCode = robot_move_as::RobotMoveResult::NO_ERROR; //return this if ultimately successful
    ROS_INFO("placing a part,  no release");
    ROS_INFO("part info: ");
    ROS_INFO_STREAM(part);
    //find the hover pose for this bin
    if (!bin_hover_jspace_pose(part.location, bin_hover_jspace_pose_)) {
        ROS_WARN("bin_hover_jspace_pose() failed for source bin %d", (int) part.location);
        errorCode = robot_move_as::RobotMoveResult::WRONG_PARAMETER;
        return errorCode;
    }
    ROS_INFO_STREAM("destination  hover pose: " << bin_hover_jspace_pose_.transpose());

    
    if (!bin_cruise_jspace_pose(part.location, bin_cruise_jspace_pose_)) {
        ROS_WARN("bin_cruise_jspace_pose() failed for location %d", (int) part.location);
        errorCode = robot_move_as::RobotMoveResult::WRONG_PARAMETER;
        return errorCode;
    }
    //compute the IK for the desired destination pose: pickup_jspace_pose_
    //first, get the equivalent desired affine of the vacuum gripper w/rt base_link;
    //need to provide the Part info and the rail displacement
    //Eigen::Affine3d RobotMoveActionServer::affine_vacuum_pickup_pose_wrt_base_link(Part part, double q_rail)
    affine_vacuum_pickup_pose_wrt_base_link_ = affine_vacuum_pickup_pose_wrt_base_link(part,
                                                                                       bin_hover_jspace_pose_[1]);
    if (!get_pickup_IK(affine_vacuum_pickup_pose_wrt_base_link_, bin_hover_jspace_pose_, dropoff_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for dropoff pose!");
        errorCode = robot_move_as::RobotMoveResult::UNREACHABLE;
        return errorCode;
    }
    ROS_INFO_STREAM("dropoff_jspace_pose_: " << dropoff_jspace_pose_.transpose());

    if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, dropoff_jspace_pose_, approach_dist_,
                             approach_dropoff_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for dropoff approach pose!");
        errorCode = robot_move_as::RobotMoveResult::UNREACHABLE;
        return errorCode;
    }
    ROS_INFO_STREAM("approach_dropoff_jspace_pose_: " << approach_dropoff_jspace_pose_.transpose());


    //start the sequence of moves:
    //ROS_INFO("moving to bin_cruise_jspace_pose_ ");
    //move_to_jspace_pose(bin_cruise_jspace_pose_, 1.0); //so far, so good, so move to cruise pose in front of bin
    //at this point, have already confired bin ID is good
    //ros::Duration(1.0).sleep();

    //now move to bin hover pose:
    ROS_INFO("moving to bin_hover_jspace_pose_ ");
    move_to_jspace_pose(bin_hover_jspace_pose_, 1.0);
    ros::Duration(1.0).sleep(); //make sure sleep is consistent w/ specified move time; min 1 sec

        //check if part is still attached
    if (!robotInterface.isGripperAttached()) {
        ROS_WARN("dropped part!");
        errorCode = robot_move_as::RobotMoveResult::PART_DROPPED; //debug--return error
        return errorCode;
    }
    
    //now move to dropoff approach pose:
    ROS_INFO("moving to approach_dropoff_jspace_pose_ ");
    move_to_jspace_pose(approach_dropoff_jspace_pose_, 1.0);
    ros::Duration(1.0).sleep();
    
    //check if part is still attached
    if (!robotInterface.isGripperAttached()) {
        ROS_WARN("dropped part!");
        errorCode = robot_move_as::RobotMoveResult::PART_DROPPED; //debug--return error
        return errorCode;
    }    

    //now move to bin dropoff pose:
    ROS_INFO_STREAM("moving to dropoff_jspace_pose_ " << std::endl << dropoff_jspace_pose_.transpose());
    move_to_jspace_pose(dropoff_jspace_pose_, 2.0); // dropoff pose
    return errorCode;
}

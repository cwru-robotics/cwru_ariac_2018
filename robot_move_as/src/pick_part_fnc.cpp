//use this fnc to pick up a specified part; it can be used by part-flipper and generic move(from,to)
// ASSUMES robot pose is already in a safe "cruise" pose, i.e. able to translate on rail without collision
// upon completion of "pick" robot will move to the pick-location's hover pose, and it will return either:
//  NO_ERROR, or:  UNREACHABLE, PART_DROPPED, WRONG_PARAMETER, or GRIPPER_FAULT
//


// use "goal", but only need to populate the "sourcePart" component
unsigned short int RobotMoveActionServer::pick_part_fnc(const robot_move_as::RobotMoveGoalConstPtr &goal) {
    unsigned short int errorCode = robot_move_as::RobotMoveResult::NO_ERROR; //return this if ultimately successful

    Part part = goal->sourcePart;
    ROS_INFO("The part is %s; it should be fetched from location code %s ", part.name.c_str(),
             placeFinder[part.location].c_str());
    ROS_INFO("part info: ");
    ROS_INFO_STREAM(part);
    //find the hover pose for this bin
    if (!hover_jspace_pose(goal->sourcePart.location, bin_hover_jspace_pose_)) {
        ROS_WARN("bin_hover_jspace_pose() failed for source bin %d", (int) goal->sourcePart.location);
        errorCode = robot_move_as::RobotMoveResult::WRONG_PARAMETER;
        return errorCode;
    }
    ROS_INFO_STREAM("bin_hover: " << bin_hover_jspace_pose_.transpose());

    
    if (!cruise_jspace_pose(goal->sourcePart.location, bin_cruise_jspace_pose_)) {
        ROS_WARN("cruise_jspace_pose() failed for location %d", (int) goal->sourcePart.location);
        errorCode = robot_move_as::RobotMoveResult::WRONG_PARAMETER;
        return errorCode;
    }
    //compute the IK for the desired pickup pose: pickup_jspace_pose_
    //first, get the equivalent desired affine of the vacuum gripper w/rt base_link;
    //need to provide the Part info and the rail displacement
    //Eigen::Affine3d RobotMoveActionServer::affine_vacuum_pickup_pose_wrt_base_link(Part part, double q_rail)
    affine_vacuum_pickup_pose_wrt_base_link_ = affine_vacuum_pickup_pose_wrt_base_link(part,
                                                                                       bin_hover_jspace_pose_[1]);
    //Eigen::Vector3d O_pickup;
    //O_pickup = affine_vacuum_pickup_pose_wrt_base_link_.translation();
    //ROS_INFO_STREAM("O_pickup: "<<O_pickup);
    //provide desired gripper pose w/rt base_link, and choose soln closest to some reference jspace pose, e.g. hover pose
    //if (!get_pickup_IK(cart_grasp_pose_wrt_base_link,approx_jspace_pose,&q_vec_soln);
    if (!get_pickup_IK(affine_vacuum_pickup_pose_wrt_base_link_, bin_hover_jspace_pose_, pickup_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for pickup pose!");
        errorCode = robot_move_as::RobotMoveResult::UNREACHABLE;
        return errorCode;
    }
    ROS_INFO_STREAM("pickup_jspace_pose_: " << pickup_jspace_pose_.transpose());

    //compute approach_pickup_jspace_pose_:  for approximate Cartesian descent and depart
    //compute_approach_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd approx_jspace_pose,double approach_dist,Eigen::VectorXd &q_vec_soln);
    if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, pickup_jspace_pose_, approach_dist_,
                             approach_pickup_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for pickup approach pose!");
        errorCode = robot_move_as::RobotMoveResult::UNREACHABLE;
        return errorCode;
    }
    ROS_INFO_STREAM("approach_pickup_jspace_pose_: " << approach_pickup_jspace_pose_.transpose());


    //start the sequence of moves:
    ROS_INFO("moving to bin_cruise_jspace_pose_ ");
    move_to_jspace_pose(bin_cruise_jspace_pose_, 1.0); //so far, so good, so move to cruise pose in front of bin
    //at this point, have already confired bin ID is good
    ros::Duration(1.0).sleep();

    //now move to bin hover pose:
    ROS_INFO("moving to bin_hover_jspace_pose_ ");
    move_to_jspace_pose(bin_hover_jspace_pose_, 1.0);
    ros::Duration(1.0).sleep(); //make sure sleep is consistent w/ specified move time; min 1 sec

    //now move to pickup approach pose:
    ROS_INFO("moving to approach_pickup_jspace_pose_ ");
    move_to_jspace_pose(approach_pickup_jspace_pose_, 1.0);
    ros::Duration(1.0).sleep();

    //ROS_INFO("enabling gripper");
    grab(); //do this early, so grasp can occur at first contact

    //now move to bin pickup pose:
    ROS_INFO_STREAM("moving to pickup_jspace_pose_ " << std::endl << pickup_jspace_pose_.transpose());
    move_to_jspace_pose(pickup_jspace_pose_, 2.0); // try to pick up part; slow approach

    errorCode = grasp_fnc(1.0); //use the grasp fnc; timeout set for 3 sec
    //ROS_INFO("got grasp error code %d",(int) errorCode);

    if (errorCode != robot_move_as::RobotMoveResult::NO_ERROR) { //if not successful, try moving to attach
        ROS_WARN("did not attach; trying lower for up to 5 sec");
        pickup_jspace_pose_[2] += 0.1; // crude move...lower via shoulder-lift joint
        bool is_attached = false;
        double t_wait_timeout = 5.0;
        double t_wait = 0.0;
        double dt_wait = 0.2;
        move_to_jspace_pose(pickup_jspace_pose_, t_wait_timeout); //5 sec is a slow move
        while ((!is_attached) && (t_wait < t_wait_timeout)) {
            is_attached = robotInterface.isGripperAttached();
            ros::Duration(dt_wait).sleep();
            t_wait += dt_wait; //keep testing attachment state; halt move as soon as attached
            ROS_INFO("waiting for gripper attachment");
        }
        if (!is_attached) {
            ROS_WARN("could not grasp part; giving up");
            release_fnc(1.0);
            //move to safe pose
            ROS_INFO("moving to hover_jspace_pose_ ");
            move_to_jspace_pose(bin_hover_jspace_pose_, 1.0);
            ros::Duration(
                    1.0).sleep(); //make sure sleep is consistent w/ specified move time; min 1 sec
            errorCode = robot_move_as::RobotMoveResult::GRIPPER_FAULT;
            return errorCode;
        }
    }

    //if here, part is grasped; now move upwards to pickup approach pose:
    ROS_INFO("part is attached to gripper");
    ROS_INFO("departing upwards to approach_pickup_jspace_pose_ ");
    move_to_jspace_pose(approach_pickup_jspace_pose_, 1.0);
    ros::Duration(1.0).sleep();

    ROS_INFO("moving to hover pose");
    move_to_jspace_pose(bin_hover_jspace_pose_, 1.0);
    ros::Duration(1.0).sleep();

    //check if part is still attached
    if (!robotInterface.isGripperAttached()) {
        ROS_WARN("dropped part!");
        errorCode = robot_move_as::RobotMoveResult::PART_DROPPED; //debug--return error
        return errorCode;
    }


    errorCode = robot_move_as::RobotMoveResult::NO_ERROR; //return success
    return errorCode;
}

//consult the "source" Part and compute if IK is realizable
unsigned short int RobotMoveActionServer::is_pickable(const robot_move_as::RobotMoveGoalConstPtr &goal) {
    unsigned short int errorCode = robot_move_as::RobotMoveResult::NO_ERROR; //return this if ultimately successful

    Part part = goal->sourcePart;
    //ROS_INFO("The part is %s; it should be fetched from location code %s ", part.name.c_str(),
    //         placeFinder[part.location].c_str());
    //ROS_INFO("part info: ");
    //ROS_INFO_STREAM(part);
    //find the hover pose for this bin
    if (!hover_jspace_pose(goal->sourcePart.location, bin_hover_jspace_pose_)) {
        ROS_WARN("bin_hover_jspace_pose() failed for source bin %d", (int) goal->sourcePart.location);
        errorCode = robot_move_as::RobotMoveResult::WRONG_PARAMETER;
        return errorCode;
    }
    //ROS_INFO_STREAM("bin_hover: " << bin_hover_jspace_pose_.transpose());

    
    if (!cruise_jspace_pose(goal->sourcePart.location, bin_cruise_jspace_pose_)) {
        ROS_WARN("cruise_jspace_pose() failed for location %d", (int) goal->sourcePart.location);
        errorCode = robot_move_as::RobotMoveResult::WRONG_PARAMETER;
        return errorCode;
    }
    //compute the IK for the desired pickup pose: pickup_jspace_pose_
    //first, get the equivalent desired affine of the vacuum gripper w/rt base_link;
    //need to provide the Part info and the rail displacement
    //Eigen::Affine3d RobotMoveActionServer::affine_vacuum_pickup_pose_wrt_base_link(Part part, double q_rail)
    affine_vacuum_pickup_pose_wrt_base_link_ = affine_vacuum_pickup_pose_wrt_base_link(part,
                                                                                       bin_hover_jspace_pose_[1]);
    //Eigen::Vector3d O_pickup;
    //O_pickup = affine_vacuum_pickup_pose_wrt_base_link_.translation();
    //ROS_INFO_STREAM("O_pickup: "<<O_pickup);
    //provide desired gripper pose w/rt base_link, and choose soln closest to some reference jspace pose, e.g. hover pose
    //if (!get_pickup_IK(cart_grasp_pose_wrt_base_link,approx_jspace_pose,&q_vec_soln);
    if (!get_pickup_IK(affine_vacuum_pickup_pose_wrt_base_link_, bin_hover_jspace_pose_, pickup_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for pickup pose!");
        errorCode = robot_move_as::RobotMoveResult::UNREACHABLE;
        return errorCode;
    }
    //ROS_INFO_STREAM("pickup_jspace_pose_: " << pickup_jspace_pose_.transpose());

    //compute approach_pickup_jspace_pose_:  for approximate Cartesian descent and depart
    //compute_approach_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd approx_jspace_pose,double approach_dist,Eigen::VectorXd &q_vec_soln);
    if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, pickup_jspace_pose_, approach_dist_,
                             approach_pickup_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for pickup approach pose!");
        errorCode = robot_move_as::RobotMoveResult::UNREACHABLE;
        return errorCode;
    }
    //ROS_INFO_STREAM("approach_pickup_jspace_pose_: " << approach_pickup_jspace_pose_.transpose());

    errorCode = robot_move_as::RobotMoveResult::NO_ERROR; //return success
    return errorCode;
}

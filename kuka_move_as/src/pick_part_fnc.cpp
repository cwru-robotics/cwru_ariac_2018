//use this fnc to pick up a specified part; it can be used by part-flipper and generic move(from,to)
// upon completion of "pick" robot will move to the pick-location's safe cruise pose, and it will return either:
//  NO_ERROR, or:  UNREACHABLE, PART_DROPPED, WRONG_PARAMETER, or GRIPPER_FAULT
//


// use "goal", but only need to populate the "sourcePart" component
unsigned short int KukaBehaviorActionServer::pick_part_fnc(const kuka_move_as::RobotBehaviorGoalConstPtr &goal) {
    unsigned short int errorCode = kuka_move_as::RobotBehaviorResult::NO_ERROR; //return this if ultimately successful

    Part part = goal->sourcePart;
    ROS_INFO("The part is %s; it should be fetched from location code %s ", part.name.c_str(),
             placeFinder[part.location].c_str());
    ROS_INFO("part info: ");
    ROS_INFO_STREAM(part);
    
    //dev/test:
    //move to hover pose for this part...SHOULD CHECK REACHABILITY FIRST
    //extract bin or box location from Part:
    int pose_code = location_to_pose_code_map[part.location];
    ROS_INFO("pose code from location code is: %d",pose_code);
    ROS_INFO("current pose code is %d",current_pose_code_);
    trajectory_msgs::JointTrajectory transition_traj;
    int npts = transitionTrajectories_.get_trajectory(current_pose_code_,pose_code, transition_traj);
    //ROS_INFO_STREAM("init traj: "<<transition_traj<<endl);
    ROS_INFO("number of points in traj = %d",npts);
    ROS_INFO("move arm to init pose: ");
    if (npts<1) {
        ROS_WARN("precomputed traj does not exist!");
      }
    else {
      ROS_INFO_STREAM("moving with traj = "<<endl<<transition_traj<<endl);
        robot_goal_.trajectory = transition_traj;
      //ROS_INFO("sending goal to arm: ");
        send_traj_goal(transition_traj);

      while (!traj_goal_complete_) {
        //put timeout here...   

        ROS_INFO("waiting for trajectory to finish...");
        ros::Duration(1.0).sleep();
        current_pose_code_ = pose_code;
       }
    }
    /*
    
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
        errorCode = kuka_move_as::RobotBehaviorResult::UNREACHABLE;
        return errorCode;
    }
    ROS_INFO_STREAM("pickup_jspace_pose_: " << pickup_jspace_pose_.transpose());

    //compute approach_pickup_jspace_pose_:  for approximate Cartesian descent and depart
    //compute_approach_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd approx_jspace_pose,double approach_dist,Eigen::VectorXd &q_vec_soln);
    if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, pickup_jspace_pose_, approach_dist_,
                             approach_pickup_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for pickup approach pose!");
        errorCode = kuka_move_as::RobotBehaviorResult::UNREACHABLE;
        return errorCode;
    }
    ROS_INFO_STREAM("approach_pickup_jspace_pose_: " << approach_pickup_jspace_pose_.transpose());


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

    if (errorCode != kuka_move_as::RobotBehaviorResult::NO_ERROR) { //if not successful, try moving to attach
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
            errorCode = kuka_move_as::RobotBehaviorResult::GRIPPER_FAULT;
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
        errorCode = kuka_move_as::RobotBehaviorResult::PART_DROPPED; //debug--return error
        return errorCode;
    }
    
    ROS_INFO("moving to bin_cruise_jspace_pose_ ");
    move_to_jspace_pose(bin_cruise_jspace_pose_, 1.0); //so far, so good, so move to cruise pose in front of bin
    //at this point, have already confired bin ID is good
    ros::Duration(1.0).sleep();    
    //check if part is still attached
    if (!robotInterface.isGripperAttached()) {
        ROS_WARN("dropped part!");
        errorCode = kuka_move_as::RobotBehaviorResult::PART_DROPPED; //debug--return error
        return errorCode;
    }
*/
    errorCode = kuka_move_as::RobotBehaviorResult::NO_ERROR; //return success
    return errorCode;
}

    /*
//consult the "source" Part and compute if IK is realizable
unsigned short int RobotMoveActionServer::is_pickable(const kuka_move_as::RobotBehaviorGoalConstPtr &goal) {
    unsigned short int errorCode = kuka_move_as::RobotBehaviorResult::NO_ERROR; //return this if ultimately successful

    Part part = goal->sourcePart;
    //ROS_INFO("The part is %s; it should be fetched from location code %s ", part.name.c_str(),
    //         placeFinder[part.location].c_str());
    //ROS_INFO("part info: ");
    //ROS_INFO_STREAM(part);
 
    //compute the IK for the desired pickup pose: pickup_jspace_pose_
    //first, get the equivalent desired affine of the vacuum gripper w/rt base_link;
    //need to provide the Part info and the rail displacement
    //Eigen::Affine3d RobotMoveActionServer::affine_vacuum_pickup_pose_wrt_base_link(Part part, double q_rail)
    affine_vacuum_pickup_pose_wrt_base_link_ = affine_vacuum_pickup_pose_wrt_base_link(part,
                                                                                       bin_hover_jspace_pose_[1]);
    Eigen::Vector3d O_pickup;
    O_pickup = affine_vacuum_pickup_pose_wrt_base_link_.translation();
    ROS_INFO_STREAM("O_pickup: "<<O_pickup.transpose());
    int ans;

    //provide desired gripper pose w/rt base_link, and choose soln closest to some reference jspace pose, e.g. hover pose
    //if (!get_pickup_IK(cart_grasp_pose_wrt_base_link,approx_jspace_pose,&q_vec_soln);
    if (!get_pickup_IK(affine_vacuum_pickup_pose_wrt_base_link_, bin_hover_jspace_pose_, pickup_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for pickup pose!");
            cout<<"is_pickable: enter 1: ";
         cin>>ans;
        errorCode = kuka_move_as::RobotBehaviorResult::UNREACHABLE;
        return errorCode;
    }
    //ROS_INFO_STREAM("pickup_jspace_pose_: " << pickup_jspace_pose_.transpose());

    //compute approach_pickup_jspace_pose_:  for approximate Cartesian descent and depart
    //compute_approach_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd approx_jspace_pose,double approach_dist,Eigen::VectorXd &q_vec_soln);
    if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, pickup_jspace_pose_, approach_dist_,
                             approach_pickup_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for pickup approach pose!");
        errorCode = kuka_move_as::RobotBehaviorResult::UNREACHABLE;
        return errorCode;
    }
    //ROS_INFO_STREAM("approach_pickup_jspace_pose_: " << approach_pickup_jspace_pose_.transpose());

    errorCode = kuka_move_as::RobotBehaviorResult::NO_ERROR; //return success
    return errorCode;
}
*/
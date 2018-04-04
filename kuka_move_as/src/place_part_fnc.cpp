//use this fnc to place up a specified, grasped part; 
// upon completion of "place" server will return either:
//  NO_ERROR, or:  UNREACHABLE, PART_DROPPED, WRONG_PARAMETER, or GRIPPER_FAULT
//


// use "goal", but only need to populate the "sourcePart" component
// specialized for placing parts in a box, either at quality-inspection station 1 or 2, as specified in goal

//XXXX EDIT ME!!!
unsigned short int KukaBehaviorActionServer::place_part_Qbox_no_release(const kuka_move_as::RobotBehaviorGoalConstPtr &goal) {
    unsigned short int errorCode = kuka_move_as::RobotBehaviorResult::NO_ERROR; //return this if ultimately successful
    trajectory_msgs::JointTrajectory transition_traj;
    inventory_msgs::Part part = goal->sourcePart;
    ROS_INFO("The part is %s; it should be fetched from location code %s ", part.name.c_str(),
            placeFinder_[part.location].c_str());
    ROS_INFO("part info: ");
    ROS_INFO_STREAM(part);

    Eigen::VectorXd approx_jspace_pose;

    //get an estimate of the correct joint-space  pose for this generic location--use the "hover" pose

    //bool KukaBehaviorActionServer::hover_jspace_pose(int8_t bin, Eigen::VectorXd &q_vec)
    if (!hover_jspace_pose(part.location, approx_jspace_pose)) {
        ROS_WARN("hover pose not recognized!");
        errorCode_ = kuka_move_as::RobotBehaviorResult::WRONG_PARAMETER;
        return errorCode_;
    }

    pickup_hover_pose_ = approx_jspace_pose; //remember this pose
    current_hover_pose_ = pickup_hover_pose_;

    //approx_jspace_pose contains pose estimate for  IKs
    //compute the IK for the desired pickup pose: pickup_jspace_pose_
    //first, get the equivalent desired affine of the vacuum gripper w/rt base_link;
    //need to provide the Part info and the rail displacement
    //Eigen::Affine3d RobotMoveActionServer::affine_vacuum_pickup_pose_wrt_base_link(Part part, double q_rail)
    affine_vacuum_pickup_pose_wrt_base_link_ = affine_vacuum_pickup_pose_wrt_base_link(part,
            approx_jspace_pose[7]);
    //provide desired gripper pose w/rt base_link, and choose soln closest to some reference jspace pose, e.g. hover pose
    //if (!get_pickup_IK(cart_grasp_pose_wrt_base_link,approx_jspace_pose,&q_vec_soln);
    if (!compute_pickup_dropoff_IK(affine_vacuum_pickup_pose_wrt_base_link_, approx_jspace_pose, pickup_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for pickup pose!");
        errorCode_ = kuka_move_as::RobotBehaviorResult::UNREACHABLE;
        return errorCode_;
    }
    desired_grasp_dropoff_pose_ = pickup_jspace_pose_;
    ROS_INFO_STREAM("pickup_jspace_pose_: " << pickup_jspace_pose_.transpose());

    //compute approach_pickup_jspace_pose_:  for approximate Cartesian descent and depart
    //compute_approach_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd approx_jspace_pose,double approach_dist,Eigen::VectorXd &q_vec_soln);
    if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, pickup_jspace_pose_, approach_dist_,
            approach_pickup_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for pickup approach pose!");
        errorCode_ = kuka_move_as::RobotBehaviorResult::UNREACHABLE;
        return errorCode_;
    }
    desired_approach_depart_pose_ = approach_pickup_jspace_pose_;
    ROS_INFO_STREAM("approach_pickup_jspace_pose_: " << approach_pickup_jspace_pose_.transpose());
    
    if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, pickup_jspace_pose_, deep_grasp_dist_,
            pickup_deeper_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for pickup approach pose!");
        errorCode = kuka_move_as::RobotBehaviorResult::UNREACHABLE;
        return errorCode;
    }
    ROS_INFO_STREAM("pickup_deeper_jspace_pose_: " << pickup_deeper_jspace_pose_.transpose());

    //dev/test:
    //move to hover pose for this part...SHOULD CHECK REACHABILITY FIRST

    //compute_pickup_dropoff_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd approx_jspace_pose,Eigen::VectorXd &q_vec_soln)

    //extract bin or box location from Part:
    int current_hover_code = location_to_pose_code_map[part.location];
    //current_hover_code is the bin-far or bin-near hover code
    ROS_INFO("pose code from location code is: %d", current_hover_code);


    if (!move_posecode1_to_posecode2(current_pose_code_, current_hover_code)) {

        ROS_WARN("error with move between pose codes");
        errorCode_ = kuka_move_as::RobotBehaviorResult::PRECOMPUTED_TRAJ_ERR; //inform our client of error code
        return errorCode_;
    }


    ROS_WARN(" DO PICKUP STEPS HERE...");

    cout<<"enter 1: ";
    cin>>ans;


    //now move to pickup approach pose:
    ROS_INFO("moving to approach_pickup_jspace_pose_ ");
    move_to_jspace_pose(approach_pickup_jspace_pose_, 1.0); //try it this way instead
    //move_to_jspace_pose(APPROACH_DEPART_CODE, 1.0); //code implies qvec in member var
    //ros::Duration(1.0).sleep();

    //ROS_INFO("enabling gripper");
    gripperInterface_.grab(); //do this early, so grasp can occur at first contact

    cout<<"enter 1: ";
    cin>>ans;
    //now move to bin pickup pose:
    ROS_INFO_STREAM("moving to pickup_jspace_pose_ " << std::endl << pickup_jspace_pose_.transpose());
    move_to_jspace_pose(pickup_jspace_pose_, 1.0); //try it this way instead
    //move_to_jspace_pose(GRASP_PLACE_CODE, 2.0); // try to pick up part; slow approach

    is_attached_ = gripperInterface_.waitForGripperAttach(1.0); //wait for grasp for  2 sec

    if (!is_attached_) {
        if (!move_into_grasp(4.0)) {
            ROS_WARN("could not grasp part; giving up");
            move_to_jspace_pose(APPROACH_DEPART_CODE, 1.0);
            //move to safe pose
            ROS_INFO("moving to hover_jspace_pose_ ");
            move_to_jspace_pose(CURRENT_HOVER_CODE, 1.0);
            current_pose_code_=current_hover_code;
  

            errorCode_ = kuka_move_as::RobotBehaviorResult::GRIPPER_FAULT;
            return errorCode_;
        }
    }
    //if here, part is attached to  gripper
    ROS_INFO("grasped part; moving to depart pose: ");
    move_to_jspace_pose(APPROACH_DEPART_CODE, 1.0);

    ROS_INFO("moving to hover_jspace_pose_ ");

    if (current_hover_code < Q1_HOVER_CODE) {
        ROS_INFO("withdrawing to nom cruise pose");
        ROS_INFO("from %d to %d ", current_hover_code, NOM_BIN_CRUISE);
        if (!move_posecode1_to_posecode2(current_hover_code, NOM_BIN_CRUISE)) {
            ROS_WARN("error with move between pose codes");
            return errorCode_;
        }
    }
    else {
            ROS_INFO("stopping at hover pose at inspection station");
            current_pose_code_=current_hover_code;
        }
        
    //check if part is still attached
    is_attached_ = gripperInterface_.isGripperAttached();
    if (!is_attached_) {
        ROS_WARN("dropped part!");
        errorCode = kuka_move_as::RobotBehaviorResult::PART_DROPPED; //debug--return error
        return errorCode;
    }
    else { ROS_INFO("part is still  attached"); }
        
        errorCode_ = kuka_move_as::RobotBehaviorResult::NO_ERROR; //return success
        return errorCode_;
}


//this version assumes the part is already grasped, and it should  be discarded

unsigned short int KukaBehaviorActionServer::discard_grasped_part() {

    trajectory_msgs::JointTrajectory transition_traj;
    if (!move_posecode1_to_posecode2(current_pose_code_, Q1_CRUISE_CODE)) {
        ROS_WARN("discard_grasped_part:  error with move between pose codes %d and %d ", current_pose_code_, Q1_CRUISE_CODE);
        return errorCode_; 
    }    
    if (!move_posecode1_to_posecode2(current_pose_code_, Q1_DISCARD_CODE)) {
        ROS_WARN("discard_grasped_part:  error with move between pose codes %d and %d ", current_pose_code_, Q1_DISCARD_CODE);
        return errorCode_;
    }

    ROS_WARN("SHOULD DO PART RELEASE HERE");
    gripperInterface_.release();
    is_attached_ = gripperInterface_.isGripperAttached();
    double timer=0;
    double dt = 0.1;
    while (timer<MAX_BEHAVIOR_SERVER_WAIT_TIME && is_attached_) { //write a fnc for this: wait_for_goal_w_timeout
        //put timeout here...   possibly return falses
        ROS_INFO("waiting for gripper release...");
        ros::Duration(dt).sleep();
        timer+=dt;
        ros::spinOnce();
        is_attached_ = gripperInterface_.isGripperAttached();
    }
    if (timer>= MAX_BEHAVIOR_SERVER_WAIT_TIME) {
        errorCode_ = errorCode_ = kuka_move_as::RobotBehaviorResult::GRIPPER_FAULT;
        return errorCode_;
    }
    

    ROS_INFO("from %d to %d ", Q1_DISCARD_CODE, Q1_CRUISE_CODE);
    if (!move_posecode1_to_posecode2(Q1_DISCARD_CODE, Q1_CRUISE_CODE)) {
        ROS_WARN("error with move between pose codes %d and %d ", Q1_DISCARD_CODE, Q1_CRUISE_CODE);
        return errorCode_;
    }


    errorCode_ = kuka_move_as::RobotBehaviorResult::NO_ERROR; //return success
    return errorCode_;
}
/*
//consult the "source" Part and compute if IK is realizable
unsigned short int RobotMoveActionServer::is_pickable(const kuka_move_as::RobotBehaviorGoalConstPtr &goal) {
unsigned short int errorCode = kuka_move_as::RobotBehaviorResult::NO_ERROR; //return this if ultimately successful

Part part = goal->sourcePart;
//ROS_INFO("The part is %s; it should be fetched from location code %s ", part.name.c_str(),
//         placeFinder_[part.location].c_str());
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
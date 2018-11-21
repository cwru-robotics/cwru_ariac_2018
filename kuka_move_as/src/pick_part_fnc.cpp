//use this fnc to pick up a specified part fromm a bin
// it can be used by part-flipper and generic move(from,to)
// upon completion of "pick" robot will move to the pick-location's safe cruise pose, and it will return either:
//  NO_ERROR, or:  UNREACHABLE, PART_DROPPED, WRONG_PARAMETER, or GRIPPER_FAULT
//


// use "goal", but only need to populate the "sourcePart" component

unsigned short int KukaBehaviorActionServer::pick_part_from_bin(const robot_behavior_interface::RobotBehaviorGoalConstPtr &goal) {
    //unsigned short int errorCode = robot_behavior_interface::RobotBehaviorResult::NO_ERROR; //return this if ultimately successful
    //trajectory_msgs::JointTrajectory transition_traj;
    int ans;
    inventory_msgs::Part part = goal->sourcePart;
    ROS_INFO("The part is %s; it should be fetched from location code %s ", part.name.c_str(),
            placeFinder_[part.location].c_str());
    ROS_INFO("part info: ");
    ROS_INFO_STREAM(part);
    trajectory_msgs::JointTrajectory traj_head,traj_tail; 
    //extract bin location from Part:
    /* do this in compute_key_poses
    int current_hover_code = location_to_pose_code_map[part.location];
    int current_cruise_code = location_to_cruise_code_map[part.location];
    
        if (!transitionTrajectories_.get_cruise_pose(part.location,current_cruise_pose_,current_cruise_code)) {
        ROS_WARN("get_cruise_pose(): bad location code!!");
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::PRECOMPUTED_TRAJ_ERR; //inform our client of error code
        return errorCode_;
    }
    Eigen::VectorXd current_bin_cruise_pose_.resize(NDOF);
    current_bin_cruise_pose_ =current_cruise_pose_; //synonym
    int current_bin_cruise_pose_code_=current_cruise_code;  
    
    transitionTrajectories_.get_hover_pose(part.location,current_hover_pose_,current_hover_code);
    */
    //Eigen::VectorXd approx_jspace_pose;
    //int current_hover_code = current_bin_hover_pose_code_;
    //int current_cruise_pose = 

    
    //unsigned short int KukaBehaviorActionServer::compute_bin_pickup_key_poses(inventory_msgs::Part part)
    errorCode_ = compute_bin_pickup_key_poses(part);   
    if (errorCode_ != robot_behavior_interface::RobotBehaviorResult::NO_ERROR) {
        
        return errorCode_;
    }


    
    //current_hover_code is the bin-far or bin-near hover code
    //ROS_INFO("pose code from location code is: %d", current_hover_code);

    
    //MOVES START HERE
    int move_to_pose_code;
    ROS_WARN("moving to respective cruise pose...");
    if (!move_posecode1_to_posecode2(current_pose_code_, current_bin_cruise_pose_code_)) {
        ROS_WARN("error with move between pose codes");
        ROS_INFO("trying to recover w/ move to closest key pose");
        /*
        if (find_nearest_key_pose(move_to_pose_code, current_key_pose_)) {
            move_to_jspace_pose(current_key_pose_, 2.5);
            current_pose_code_ = move_to_pose_code;
            ROS_INFO("current_pose_code_ = %d",current_pose_code_);
            
        }
         * */
            //try again:
        if (!move_posecode1_to_posecode2(current_pose_code_, current_bin_cruise_pose_code_)) {
            ROS_WARN("error with move between pose codes; recovery failed");   
            errorCode_ = robot_behavior_interface::RobotBehaviorResult::PRECOMPUTED_TRAJ_ERR; //inform our client of error code
            return errorCode_;
        }
        
    }
    if (bad_state_ ==rtn_state_) {
        ROS_WARN("TRYING TO RECOVER FROM ABORT");
        ros::Duration(1.0).sleep();
        move_to_jspace_pose(current_bin_cruise_pose_, 8.0);     
    }    
    
    //try combining moves: current_hover_pose_, approach_pickup_jspace_pose_
    traj_head = jspace_pose_to_traj(current_hover_pose_,1.5);
    traj_tail = jspace_pose_to_traj(desired_approach_jspace_pose_,1.0);
    
    traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories
    ROS_INFO("sending multi-point concatenated traj: ");
    
    send_traj_goal(traj_head,CUSTOM_JSPACE_POSE);
    
    /*
    ROS_WARN("moving to respective hover pose");
    move_to_jspace_pose(current_hover_pose_, 3.5); 
    if (bad_state_ ==rtn_state_) {
        ROS_WARN("TRYING TO RECOVER FROM ABORT");
        ros::Duration(1.0).sleep();
        move_to_jspace_pose(current_hover_pose_, 5.0);     
    }     
    
    
    //now move to pickup approach pose:
    ROS_INFO("moving to approach_pickup_jspace_pose_ ");
    move_to_jspace_pose(approach_pickup_jspace_pose_, 3.0); //try it this way instead
    //move_to_jspace_pose(APPROACH_DEPART_CODE, 1.0); //code implies qvec in member var
    //ros::Duration(1.0).sleep();
    if (bad_state_ ==rtn_state_) {
        ROS_WARN("TRYING TO RECOVER FROM ABORT");
        ros::Duration(1.0).sleep();
        move_to_jspace_pose(approach_pickup_jspace_pose_, 5.0);     
    } 
    */
    ROS_WARN(" DO PICKUP STEPS HERE...");
    
    //ROS_INFO("enabling gripper");
    gripperInterface_.grab(); //do this early, so grasp can occur at first contact
    is_attached_ =  false;
    ROS_INFO("descending to grasp pose, ");
    //cout<<"ready to descend to grasp; enter 1: ";
    //cin>>ans;
    //now move to bin pickup pose:
    //ROS_INFO_STREAM("moving to pickup_jspace_pose_ " << std::endl << pickup_jspace_pose_.transpose());
    move_into_grasp(pickup_jspace_pose_, 1.5); //provide target pose
    ROS_INFO("at computed grasp pose;checking for gripper attachment ");    
    is_attached_ = gripperInterface_.waitForGripperAttach(2.0); //wait for grasp for a bit


    if (!is_attached_) {
        ROS_WARN("did not attach; attempting to descend further: ");
        //cin>>ans;
        if (!move_into_grasp(MOVE_INTO_GRASP_TIME)) {
            ROS_WARN("could not grasp part; giving up; moving to depart pose...");
            move_to_jspace_pose(desired_depart_jspace_pose_, 1.5); //approach_pickup_jspace_pose_

            ROS_INFO("moving to current_hover_pose_ ");//pickup_hover_pose_
            //move_to_jspace_pose(CURRENT_HOVER_CODE, 1.0);
            move_to_jspace_pose(current_bin_hover_pose_, 2.0); //try it this way instead     
            if (bad_state_ ==rtn_state_) {
                ROS_WARN("TRYING TO RECOVER FROM ABORT");
                ros::Duration(1.0).sleep();
                move_to_jspace_pose(current_bin_hover_pose_, 5.0);     
            }             
            current_pose_code_=current_bin_hover_pose_code_; //establish code for recognized, key pose
            ROS_INFO("moving to current_cruise_pose_ ");            
            move_to_jspace_pose(current_bin_cruise_pose_, 3);  
            if (bad_state_ ==rtn_state_) {
                ROS_WARN("TRYING TO RECOVER FROM ABORT");
                ros::Duration(1.0).sleep();
                move_to_jspace_pose(current_bin_cruise_pose_, 10.0);     
            }                
            current_pose_code_ = current_bin_cruise_pose_code_; //keep track of where we are, in terms of pose codes
            
            errorCode_ = robot_behavior_interface::RobotBehaviorResult::GRIPPER_FAULT;
            return errorCode_;
        }
    }
    //if here, part is attached to  gripper 
    //combine these  moves: approach_pickup_jspace_pose_, computed_bin_escape_jspace_pose_,
    //current_bin_cruise_pose_ w/ frozen wrist, current_bin_cruise_pose_
    traj_head = jspace_pose_to_traj(desired_depart_jspace_pose_,1.0);

    
    
//    send_traj_goal(transition_traj,CUSTOM_JSPACE_POSE);    
    
    /**/
    ROS_INFO("grasped part; moving to depart pose; enter 1: ");//approach_pickup_jspace_pose_
    //cin>>ans;
    
    /*
    move_to_jspace_pose(approach_pickup_jspace_pose_, 2.0);    
    if (bad_state_ ==rtn_state_) {
        ROS_WARN("TRYING TO RECOVER FROM ABORT");
        ros::Duration(1.0).sleep();
        move_to_jspace_pose(approach_pickup_jspace_pose_, 5.0);     
    } 
    */
     //move_to_jspace_pose(computed_jspace_approach_, 1.0);   
    //cout<<"ready to move to hover pose; enter 1: ";
    //cin>>ans; 
    
    ROS_INFO("moving to hover_jspace_pose_ ");
    //freeze wrist:
    for (int i=4;i<6;i++) current_hover_pose_[i] = desired_depart_jspace_pose_[i];
    traj_tail = jspace_pose_to_traj(current_hover_pose_,1.0);    
    traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories
    /*
    move_to_jspace_pose(current_hover_pose_, 2); 
    if (bad_state_ ==rtn_state_) {
        ROS_WARN("TRYING TO RECOVER FROM ABORT");
        ros::Duration(1.0).sleep();
        move_to_jspace_pose(current_hover_pose_, 5.0);     
    } 
    */
    //if (current_hover_code < Q1_HOVER_CODE) {
    //    ROS_INFO("withdrawing to nom cruise pose");
    //    ROS_INFO("from %d to %d ", current_hover_code, NOM_BIN_CRUISE);
    
    ROS_WARN("moving to computed_bin_escape_jspace_pose_; enter 1: ");
    //cin>>ans;
    //freeze wrist
    for (int i=4;i<6;i++) computed_bin_escape_jspace_pose_[i] = desired_depart_jspace_pose_[i];
    traj_tail = jspace_pose_to_traj(computed_bin_escape_jspace_pose_,1.0);    
    traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories  
    /*
    move_to_jspace_pose(computed_bin_escape_jspace_pose_, 2.0);  
    if (bad_state_ ==rtn_state_) {
        ROS_WARN("TRYING TO RECOVER FROM ABORT");
        ros::Duration(1.0).sleep();
        move_to_jspace_pose(computed_bin_escape_jspace_pose_, 5.0);     
    } 
    */ 
    //modify J1-ang only to get to a cruise pose:

    //try cutting this move:
    //ROS_INFO("moving to  cruise pose; enter 1");
    //cin>>ans;
    //move_to_jspace_pose(computed_bin_cruise_jspace_pose_, 1.0);   //current_cruise_pose_ instead?  

    //xxx still aborted at 4.0 sec
    ROS_INFO("moving to current_bin_cruise_pose_ cruise pose; enter 1");
    //cin>>ans;   
    //riskier--freeze toolflange
    Eigen::VectorXd temp_pose;
    temp_pose.resize(8);
    temp_pose = current_bin_cruise_pose_; //move here in 2 steps
    //keep toolflange the same
    for (int i=4;i<6;i++) temp_pose[i] = desired_depart_jspace_pose_[i];
    traj_tail = jspace_pose_to_traj(temp_pose,1.5);    
    traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories  
    /*
    //watch out for bent wrist:
    if (temp_pose[5]<-1.57) temp_pose[5]=-1.57;
    if (temp_pose[5]>1.57) temp_pose[5]=1.57;
    
    ROS_INFO("moving to temp pose");
    move_to_jspace_pose(temp_pose, 6.0);   
    if (bad_state_ ==rtn_state_) {
        ROS_WARN("TRYING TO RECOVER FROM ABORT");
        ros::Duration(1.0).sleep();
        move_to_jspace_pose(temp_pose, 10.0);   
    }
    if (bad_state_ ==rtn_state_) {
        ROS_WARN("TRYING TO RECOVER FROM ABORT");
        move_to_jspace_pose(temp_pose, 10.0);   
    }    
    */
    //current_bin_cruise_pose_[6] = approach_pickup_jspace_pose_[6];
    ROS_INFO("finishing move to current_bin_cruise_pose_");
    traj_tail = jspace_pose_to_traj(current_bin_cruise_pose_,1);    
    traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories     
    /* 
    move_to_jspace_pose(current_bin_cruise_pose_, 3.0);     
    current_pose_code_ = current_bin_cruise_pose_code_; //keep track of where we are, in terms of pose codes
    if (bad_state_ ==rtn_state_) {
        ROS_WARN("TRYING TO RECOVER FROM ABORT");
        ros::Duration(1.0).sleep();
        move_to_jspace_pose(current_bin_cruise_pose_, 5.0);     
 
    }    
     * */
    current_pose_code_ = current_bin_cruise_pose_code_;
    ROS_INFO("sending multi-point concatenated traj: ");
    send_traj_goal(traj_head,current_bin_cruise_pose_code_);
    
    //CAREFUL: CHANGE IN STRATEGY; END THIS FUNC AT BIN CRUISE POSE
    //move to box cruise pose:
    /*
    if (!move_posecode1_to_posecode2(current_pose_code_, Q1_CRUISE_CODE)) {
        ROS_WARN("error with move between pose codes");
    }
    if (bad_state_ ==rtn_state_) {
        ROS_WARN("TRYING TO RECOVER FROM ABORT");
        ros::Duration(1.0).sleep();
        move_to_jspace_pose(q1_cruise_pose_, 5.0);     
    }        //try again, if necessary
    if (bad_state_ ==rtn_state_) {
        ROS_WARN("TRYING TO RECOVER FROM ABORT");
        ros::Duration(1.0).sleep();
        move_to_jspace_pose(q1_cruise_pose_, 5.0);     
    } 
     * */
    //check if part is still attached
    is_attached_ = gripperInterface_.isGripperAttached();
    if (!is_attached_) {
        ROS_WARN("dropped part!");
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::PART_DROPPED; //debug--return error
        return errorCode_;
    }
    else { ROS_INFO("part is still  attached"); }
        
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::NO_ERROR; //return success
        return errorCode_;
}




//must compute pickup_deeper_jspace_pose_ first!!
//calls for robot to move towards pickup_deeper_jspace_pose_, testing gripper status along the way
//aborts when attachment is detected
//internal func, NOT a behavior!
//does  NOT return status to client.  Need to do this separately
bool KukaBehaviorActionServer::move_into_grasp(double arrival_time) {
    ROS_INFO_STREAM("MOVE TO DEEP GRASP, pose = "<<endl<<pickup_deeper_jspace_pose_.transpose()<<endl);
    trajectory_msgs::JointTrajectory transition_traj;
    //move slowly--say 5 seconds to press into part
    transition_traj = jspace_pose_to_traj(pickup_deeper_jspace_pose_,arrival_time);
    errorCode_ = robot_behavior_interface::RobotBehaviorResult::NO_ERROR;
    //ROS_INFO_STREAM("transition traj = "<<endl<<transition_traj<<endl);
    is_attached_ = false;
    send_traj_goal(transition_traj);  //start the motion
    while (!traj_goal_complete_ && !is_attached_) { //write a fnc for this: wait_for_goal_w_timeout
        //put timeout here...   possibly return falses
        ROS_INFO("waiting for grasp...");
        ros::Duration(0.1).sleep();
        ros::spinOnce();
        is_attached_ = gripperInterface_.isGripperAttached();
        if (is_attached_) traj_ctl_ac_.cancelGoal();
    }
    if (!is_attached_)  {
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::GRIPPER_FAULT;
        return false;
    }
    ROS_WARN("part is attached");
    return true;

}

bool KukaBehaviorActionServer::move_into_grasp(Eigen::VectorXd pickup_jspace_pose, double arrival_time) {
    ROS_INFO_STREAM("MOVE TO GRASP, pose = "<<endl<<pickup_jspace_pose.transpose()<<endl);
    trajectory_msgs::JointTrajectory transition_traj;
    //move slowly--say 5 seconds to press into part
    transition_traj = jspace_pose_to_traj(pickup_jspace_pose,arrival_time);
    errorCode_ = robot_behavior_interface::RobotBehaviorResult::NO_ERROR;
    //ROS_INFO_STREAM("transition traj = "<<endl<<transition_traj<<endl);
    is_attached_ = false;
    send_traj_goal(transition_traj);  //start the motion
    while (!traj_goal_complete_ && !is_attached_) { //write a fnc for this: wait_for_goal_w_timeout
        //put timeout here...   possibly return falses
        ROS_INFO("waiting for grasp...");
        ros::Duration(0.1).sleep();
        ros::spinOnce();
        is_attached_ = gripperInterface_.isGripperAttached();
        if (is_attached_) traj_ctl_ac_.cancelGoal();
    }
    if (!is_attached_)  {
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::GRIPPER_FAULT;
        return false;
    }
    ROS_WARN("part is attached");
    return true;

}


//pick part from box (and discard)
//would use this e.g. to remove bad part at Q2, or to make an order-update change, or to unpack a box for priority shipment
// presumed initial state of robot is some cruise pose; COULD  be grasping a different part, 
//  do  NOT leave robot over a box; if adjusting part locations, return to cruise when done

//break this into multiple steps:
//1) compute key poses for box pickup
//     unsigned short int KukaBehaviorActionServer::alt_compute_box_dropoff_key_poses(inventory_msgs::Part part) {
//2) if currently grasping a part, discard it
//3) transition to box hover pose,  regardless of initial condition
//4) go  through steps to grasp part
//   --> if unsuccessful, return gripper failure;
//5) with part grasped, use discard_grasped_part fnc

//fnc:  transition_to_box_cruise_pose(box_cruise_pose)
//fnc:  transition_to_box_hover_pose(box_hover_pose)

/*
void KukaBehaviorActionServer::transition_to_box_cruise_pose(box_cruise_pose) {
    get_fresh_joint_states(); //update joint_state_vec_
    //note: computation of box_cruise_pose takes into account initial pose of J1; 
    //presumably, initial pose is a cruise pose, so should  be safe to move to box_cruise_pose from here;

}
*/

unsigned short int  KukaBehaviorActionServer::pick_part_from_box(Part part, double timeout) {
    ROS_INFO("The part is %s; it should be fetched from location code %s ", part.name.c_str(),
            placeFinder_[part.location].c_str());
    trajectory_msgs::JointTrajectory traj_head,traj_tail; 

    // set these member var values:
    //current_hover_pose_
    //approach_dropoff_jspace_pose_ = desired_approach_depart_pose_
    //desired_grasp_dropoff_pose_
    //pickup_deeper_jspace_pose_
    
    //XXXXXXXXXXXXXXX  Q1  ONLY XXXXXXXXXXXXXXXXXX
    //BUG WORK-AROUND: Assign location code Q1;
    //part.location = inventory_msgs::Part::QUALITY_SENSOR_1;
    //XXXXXXXXXXXXXXXXXX
    
    //compute these poses:      
    //box_dropoff_cruise_pose_, box_dropoff_hover_pose_=box_cam_grasp_inspection_pose_, desired_grasp_dropoff_pose_, approach_dropoff_jspace_pose_, 

    errorCode_ = alt_compute_box_dropoff_key_poses(part);
    if (errorCode_ != robot_behavior_interface::RobotBehaviorResult::NO_ERROR) {
        return errorCode_;
    }
    desired_approach_jspace_pose_=approach_dropoff_jspace_pose_; //synonym...for pickup of part from box, not dropoff 
    //extract box location codes from Part: obsolete
    /*
    int current_hover_code = location_to_pose_code_map[part.location];
    int current_cruise_code = location_to_cruise_code_map[part.location];
    
    //get actual q_vecs for current cruise and hover poses
    if (!transitionTrajectories_.get_cruise_pose(part.location,current_cruise_pose_,current_cruise_code)) {
        ROS_WARN("get_cruise_pose(): bad location code!!");
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::PRECOMPUTED_TRAJ_ERR; //inform our client of error code
        return errorCode_;
    }
    
    transitionTrajectories_.get_hover_pose(part.location,current_hover_pose_,current_hover_code);    
    //now move to approach_dropoff_jspace_pose_:
    //ROS_INFO("moving to approach_dropoff_jspace_pose_ ");
    //cout<<"enter 1: ";
    //cin>>ans;
    move_to_jspace_pose(approach_dropoff_jspace_pose_, 1.0); //try it this way instead    
    if (bad_state_ ==rtn_state_) {
        ROS_WARN("TRYING TO RECOVER FROM ABORT");
        ros::Duration(1.0).sleep();
        move_to_jspace_pose(approach_dropoff_jspace_pose_, 5.0);     
    }     
    */
    //note: computation of box_cruise_pose takes into account initial pose of J1; 
    //presumably, initial pose is a cruise pose, so should  be safe to move to box_cruise_pose from here;    
    ROS_INFO("enabling gripper");
    gripperInterface_.grab(); //do this early, so grasp can occur at first contact
    is_attached_ =  false;

//xxx: this is last print-out seen before Eigen dimension complaint and crash xxxx
    
    //construct a trajectory from cruise->hover->approach->grasp pose
    double move_time_est = estimate_move_time(joint_state_vec_,box_dropoff_cruise_pose_)+1.0;     
    traj_head = jspace_pose_to_traj(box_dropoff_cruise_pose_,move_time_est); 
    int ans;
    //cout<<"enter 1: ";
    //cin>>ans;
        
    move_time_est = estimate_move_time(box_dropoff_cruise_pose_,box_cam_grasp_inspection_pose_)+1.0;     
    //    cout<<"enter 2: ";
    //cin>>ans;    
    traj_tail = jspace_pose_to_traj(box_cam_grasp_inspection_pose_,move_time_est); 
    //    cout<<"enter 3: ";
    //cin>>ans;    
    traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail);
    //    cout<<"enter 4: ";
    //cin>>ans;    
    move_time_est = estimate_move_time(box_cam_grasp_inspection_pose_, desired_approach_jspace_pose_)+0.5;
    //    cout<<"enter 5: ";
    //cin>>ans;    
    traj_tail = jspace_pose_to_traj(desired_approach_jspace_pose_, move_time_est);
    traj_head = transitionTrajectories_.concat_trajs(traj_head, traj_tail); //concatenate trajectories 
    move_time_est = estimate_move_time(approach_dropoff_jspace_pose_, desired_grasp_dropoff_pose_) + 0.5;
    traj_tail = jspace_pose_to_traj(desired_grasp_dropoff_pose_, move_time_est);
    traj_head = transitionTrajectories_.concat_trajs(traj_head, traj_tail); //concatenate trajectories     

    send_traj_goal(traj_head, CUSTOM_JSPACE_POSE);      
    //    cout<<"enter 6: ";
    //cin>>ans;
    move_into_grasp(desired_grasp_dropoff_pose_, 1.5); //provide target pose
    cout<<"at computed grasp pose; "<<endl;    
    is_attached_ = gripperInterface_.waitForGripperAttach(2.0); //wait for grasp for a bit
    //descend further while testing for part attachment:
    if (!move_into_grasp(MOVE_INTO_GRASP_TIME)) {
            ROS_WARN("could not grasp part; giving up; moving to approach pose...");
            move_to_jspace_pose(approach_dropoff_jspace_pose_, 1.0); //
            if (bad_state_ ==rtn_state_) {
                ROS_WARN("TRYING TO RECOVER FROM ABORT");
                ros::Duration(1.0).sleep();
                move_to_jspace_pose(approach_dropoff_jspace_pose_, 5.0);     
            }  
            ROS_INFO("moving to current_hover_pose_ ");//pickup_hover_pose_
            //move_to_jspace_pose(CURRENT_HOVER_CODE, 1.0);
            move_to_jspace_pose(current_hover_pose_, 1.0); //try it this way instead   
            if (bad_state_ ==rtn_state_) {
                ROS_WARN("TRYING TO RECOVER FROM ABORT");
                ros::Duration(1.0).sleep();
                move_to_jspace_pose(current_hover_pose_, 5.0);     
            }              
            //current_pose_code_=current_hover_code; //establish code for recognized, key pose
            errorCode_ = robot_behavior_interface::RobotBehaviorResult::GRIPPER_FAULT;
            return errorCode_;

    }
    //if here, part is attached to  gripper;
    ROS_INFO("discarding grasped part...");
    discard_grasped_part(part);
    /*
    
    ROS_INFO("grasped part; moving to depart pose: "); 
    move_to_jspace_pose(approach_dropoff_jspace_pose_, 1.0);    
    
    ROS_INFO("done w/ pick_part_from_box; still in approach pose");
    //ROS_INFO("moving to current_hover_pose_ ");//pickup_hover_pose_
    //move_to_jspace_pose(CURRENT_HOVER_CODE, 1.0);
    //move_to_jspace_pose(current_hover_pose_, 1.0); //try it this way instead       
    //current_pose_code_=current_hover_code; //establish code for recognized, key pose
     * */
    errorCode_ = robot_behavior_interface::RobotBehaviorResult::NO_ERROR;
            return errorCode_;
}


//this version assumes the part is already grasped, and it should  be discarded
/*
unsigned short int KukaBehaviorActionServer::discard_grasped_part() {

    trajectory_msgs::JointTrajectory transition_traj;
    if (!move_posecode1_to_posecode2(current_pose_code_, Q1_DISCARD_CODE)) {
        ROS_WARN("discard_grasped_part:  error with move between pose codes %d and %d ", current_pose_code_, Q1_DISCARD_CODE);
        return errorCode_;
    }

    ROS_WARN("SHOULD DO PART RELEASE HERE");
    //cout<<"enter 1: ";
    //cin>>g_ans;

    ROS_INFO("from %d to %d ", Q1_DISCARD_CODE, Q1_CRUISE_CODE);
    if (!move_posecode1_to_posecode2(Q1_DISCARD_CODE, Q1_CRUISE_CODE)) {
        ROS_WARN("error with move between pose codes %d and %d ", Q1_DISCARD_CODE, Q1_CRUISE_CODE);
        return errorCode_;
    }


    errorCode_ = robot_behavior_interface::RobotBehaviorResult::NO_ERROR; //return success
    return errorCode_;
}*/
/*
//consult the "source" Part and compute if IK is realizable
unsigned short int RobotMoveActionServer::is_pickable(const robot_behavior_interface::RobotBehaviorGoalConstPtr &goal) {
unsigned short int errorCode = robot_behavior_interface::RobotBehaviorResult::NO_ERROR; //return this if ultimately successful

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
    errorCode_ = robot_behavior_interface::RobotBehaviorResult::UNREACHABLE;
    return errorCode_;
}
//ROS_INFO_STREAM("pickup_jspace_pose_: " << pickup_jspace_pose_.transpose());

//compute approach_pickup_jspace_pose_:  for approximate Cartesian descent and depart
//compute_approach_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd approx_jspace_pose,double approach_dist,Eigen::VectorXd &q_vec_soln);
if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, pickup_jspace_pose_, approach_dist_,
                         approach_pickup_jspace_pose_)) {
    ROS_WARN("could not compute IK soln for pickup approach pose!");
    errorCode_ = robot_behavior_interface::RobotBehaviorResult::UNREACHABLE;
    return errorCode_;
}
//ROS_INFO_STREAM("approach_pickup_jspace_pose_: " << approach_pickup_jspace_pose_.transpose());

errorCode = robot_behavior_interface::RobotBehaviorResult::NO_ERROR; //return success
return errorCode;
}
 */

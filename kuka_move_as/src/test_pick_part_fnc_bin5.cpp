//specialized fnc to develop grasp for bin5
// use "goal", but only need to populate the "sourcePart" component
//replaced "deep"  and "near" with wrist-flip observation
//const double X_DEEP = -0.83;
//see what happens w/ middle strategy on front row:
//const double X_NEAR = -0.6; //-0.7;  //-0.73 is nom midpoint; try near up to -0.7

//cruise is at J1 = pi/2 for BIN1-> BIN3, but at -pi/2 for Bin4,  Bin5
//approach ang is ~ +0.3 for bin1 =~ 1.57 - 1.3

//   spin J1 to -pi/2, then continue to about -0.3 (near zero)
// BIN4, BIN5: rotation J1 is positive to go from cruise to bin-reach pose
// this requires an overhead flip of J2 and J4 and J6

unsigned short int KukaBehaviorActionServer::test_pick_part_from_bin5(const robot_behavior_interface::RobotBehaviorGoalConstPtr &goal) {
    int ans;
    inventory_msgs::Part part = goal->sourcePart;
    ROS_INFO("The part is %s; it should be fetched from location code %s ", part.name.c_str(),
            placeFinder_[part.location].c_str());
    ROS_INFO("part info: ");
    ROS_INFO_STREAM(part);
    trajectory_msgs::JointTrajectory traj_head, traj_tail;
    Eigen::VectorXd q_vec, old_q_vec;
    q_vec.resize(NDOF);
    old_q_vec.resize(NDOF);
    double move_time_est;
    double J1_ang, d8_disp;
    double delta_d8, delta_J1;
    bool wrist_flip = false;

    //COMPUTE KEY POSES FOR MANIPULATION HERE:
    errorCode_ = alt_compute_bin_pickup_key_poses(part);
    if (errorCode_ != robot_behavior_interface::RobotBehaviorResult::NO_ERROR) {
        return errorCode_;
    }


    //XXX CHECK THIS FOR BINS 4 and 5
    if (pickup_jspace_pose_[5] < 0.0) wrist_flip = true; //use this for "deep" grasp solns

    // different strategy if "deep", i.e. part is in back row;
    bool is_deep = false;
    bool is_near = true;
    bool is_middle = false;
    double x_part = part.pose.pose.position.x;
    double y_part = part.pose.pose.position.y;
    Eigen::VectorXd bin5_deep_cruise_pose;
    transitionTrajectories_.c_array_to_qvec(BIN1_DEEP_CRUISE_array, bin5_deep_cruise_pose);
    bin5_deep_cruise_pose[0] = -1.57; //lefty/righty swap
    //d8 is very wrong, but will fix that...
    q_vec = bin5_deep_cruise_pose;

    if (x_part < X_DEEP) {
        is_deep = true;
        is_near = false;
    } else if (x_part > X_NEAR) {
        is_deep = false;
        is_near = true;
    } else {
        is_deep = false;
        is_near = false;
        is_middle = true;
    }
    //DO APPROACH:
    //ros::spinOnce();
    get_fresh_joint_states(); //update joint_state_vec_
    //if (is_deep) {
    if (wrist_flip) {
        ROS_INFO("back-row; computing insertion motion");
        //place the start here, at this cruise pose:
        //XXX NEED A NEW REFERENCE POSE HERE...
        //transitionTrajectories_.c_array_to_qvec(BIN5_DEEP_CRUISE_array,q_vec);
        //need to assure elbow clearance
        get_fresh_joint_states(); // want to update joint states
        
        if (desired_approach_jspace_pose_[1]> -1.35) {
            ROS_WARN("desired approach shoulder ang > -1.35; will NOT correct this... ");
            //desired_approach_jspace_pose_[1] = -1.35;
        }
        //XXX FIX ME!
        //if (y_part> -0.35) y_part = -0.35; //GENERALIZE THIS TO OTHER BINS

        //negate R_OUTSTRETCHED for approach to bin5 or bin4:
        d8_disp = y_part - R_OUTSTRETCHED - Y_BASE_WRT_WORLD_AT_D8_HOME;
        ROS_INFO("compute d8 = %f", d8_disp);
        q_vec[7] = d8_disp;
        //q_vec[7] = d8_disp;  //correct the rail displacement, per computation
        J1_ang = q_vec[0]; //SHOULD BE -pi/2
        //J1 = pi/2 at cruise pose;  want J1<pi/2 for rotation to bin5:
        delta_J1 = (desired_approach_jspace_pose_[0] - J1_ang) / 10.0; //this will be positive
        //compute move time to get to this pose:
        //XXX NOTE: IF RIGHTY/LEFT SWAP, NEED INTERMEDIATE POSES!!!
        bin5_deep_cruise_pose[7] = d8_disp;
        goto_cruise_pose(bin5_deep_cruise_pose);

        move_time_est = estimate_move_time(joint_state_vec_, q_vec) + 1;
        ROS_INFO("estimated move time to cruise start = %f", move_time_est);
        traj_head = jspace_pose_to_traj(q_vec, move_time_est);
        send_traj_goal(traj_head, CUSTOM_JSPACE_POSE); //send this now--may need to confirm it is successful
        //XXX FIX  ME!

        get_fresh_joint_states(); //update joint state vec
        //check that robot is converged:
        //IF THIS IS SUCCESSFUL,  PROPAGATE IT TO OTHER CASES
        move_time_est = 1.0;
        while (move_time_est > 0.02) {
            get_fresh_joint_states(); //refresh joint states
            move_time_est = estimate_move_time(joint_state_vec_, q_vec);
            ROS_INFO("settling: move_time_est = %f", move_time_est);
            traj_head = jspace_pose_to_traj(q_vec, move_time_est);
            send_traj_goal(traj_head, CUSTOM_JSPACE_POSE);
        }
        ROS_INFO("cruise pose has settled; start deep reach into bin");
        //reset the head of the trajectory:
        move_time_est = estimate_move_time(joint_state_vec_, q_vec) + 0.1;
        traj_head = jspace_pose_to_traj(q_vec, move_time_est);

        //move forward, computing coordinated J1/d8 for pure x motion (const y)       
        for (int i = 0; i < 10; i++) {
            old_q_vec = q_vec;
            J1_ang += delta_J1; //J1 will be increasing
            delta_d8 = R_OUTSTRETCHED * cos(J1_ang) * delta_J1; //d8 will be increasing
            d8_disp += delta_d8;
            q_vec[0] = J1_ang;
            q_vec[7] = d8_disp;
            move_time_est = estimate_move_time(old_q_vec, q_vec) + 0.1;
            traj_tail = jspace_pose_to_traj(q_vec, move_time_est);
            traj_head = transitionTrajectories_.concat_trajs(traj_head, traj_tail); //concatenate trajectories 
        }
        //finish up at approach pose:
        move_time_est = estimate_move_time(q_vec, desired_approach_jspace_pose_) + 1;
        traj_tail = jspace_pose_to_traj(desired_approach_jspace_pose_, move_time_est);
        traj_head = transitionTrajectories_.concat_trajs(traj_head, traj_tail); //concatenate trajectories             

        send_traj_goal(traj_head, CUSTOM_JSPACE_POSE);
        //ROS_INFO("pausing at approach pose: ");
        //ros::Duration(2.0).sleep();
        ROS_INFO("sent multipoint traj to approach pose for part in back row");
    }
    else {
        ROS_WARN("acquiring part from front or middle row of bin...");

        transitionTrajectories_.c_array_to_qvec(BIN1_CENTER_ROW_CRUISE_array, q_vec);
        //q_vec[0]= -1.57; // change RIGHTY/LEFT for BIN4/BIN5 approach
        goto_cruise_pose(bin5_deep_cruise_pose);

        if (desired_approach_jspace_pose_[1]> -1.35) {
            ROS_WARN("desired approach shoulder ang > -1.35; will NOT correct this... ");
            //desired_approach_jspace_pose_[1] = -1.35;
        }

        // FIX ME!!  reference y-value to current bin
        //if (y_part> -0.35) y_part = -0.35;
        d8_disp = y_part - R_OUTSTRETCHED - Y_BASE_WRT_WORLD_AT_D8_HOME;
        //q_vec[7] = d8_disp;
        bin5_deep_cruise_pose[7] = d8_disp;
        goto_cruise_pose(bin5_deep_cruise_pose);
        q_vec = bin5_deep_cruise_pose;
        //q_vec[7] = d8_disp;  //correct the rail displacement, per computation
        J1_ang = q_vec[0];
        delta_J1 = (desired_approach_jspace_pose_[0] - J1_ang) / 10.0; //positive increments
        //compute move time to get to this pose:
        
        move_time_est = estimate_move_time(joint_state_vec_, q_vec) + 0.5;
        traj_head = jspace_pose_to_traj(q_vec, move_time_est);
        //send_traj_goal(traj_head,CUSTOM_JSPACE_POSE);
        //cout<<"enter  1: ";
        //cin>>ans;

        //move forward, computing coordinated J1/d8 for pure x motion (const y)       
        for (int i = 0; i < 10; i++) {
            old_q_vec = q_vec;
            J1_ang += delta_J1;
            delta_d8 = R_OUTSTRETCHED * cos(J1_ang) * delta_J1;
            d8_disp += delta_d8;
            q_vec[0] = J1_ang;
            q_vec[7] = d8_disp;
            move_time_est = estimate_move_time(old_q_vec, q_vec) + 0.1;
            traj_tail = jspace_pose_to_traj(q_vec, move_time_est);
            traj_head = transitionTrajectories_.concat_trajs(traj_head, traj_tail); //concatenate trajectories 
        }
        //finish up at approach pose:
        move_time_est = estimate_move_time(q_vec, desired_approach_jspace_pose_) + 1;
        traj_tail = jspace_pose_to_traj(desired_approach_jspace_pose_, move_time_est);
        traj_head = transitionTrajectories_.concat_trajs(traj_head, traj_tail); //concatenate trajectories             


        send_traj_goal(traj_head, CUSTOM_JSPACE_POSE);
        ROS_INFO("sent multipoint traj to approach pose for part in middle row");
    }

    ROS_WARN(" DO PICKUP STEPS HERE...");

    ROS_INFO("descending to grasp pose, ");

    gripperInterface_.grab(); //do this early, so grasp can occur at first contact
    is_attached_ = false;
    ROS_INFO("descending to grasp pose, ");

    move_into_grasp(pickup_jspace_pose_, 2.0); //provide target pose
    ROS_INFO("at computed grasp pose;checking for gripper attachment ");
    is_attached_ = gripperInterface_.waitForGripperAttach(2.0); //wait for grasp for a bit


    if (!is_attached_) move_into_grasp(MOVE_INTO_GRASP_TIME);


    //EXTRACT ARM FROM BIN:
    if (wrist_flip) {
        //else if (is_deep) {
        ROS_WARN("extracting arm from back row of bin");
        traj_head = jspace_pose_to_traj(desired_approach_jspace_pose_, 0.8);
        //transitionTrajectories_.c_array_to_qvec(BIN1_DEEP_VIA_2,q_vec);
        q_vec = desired_approach_jspace_pose_;

        // NOT SO GOOD--elbow hits; need elbow straighter
        //outstretched arm:
        //[0.172, -1.3,        0,  0.3,   -0.043,   -1.4,   2.945,  -1.338
        q_vec[1] = -1.35;
        q_vec[3] = 0.3;
        q_vec[5] = -1.4;
        move_time_est = estimate_move_time(desired_approach_jspace_pose_, q_vec) + 0.5;

        traj_tail = jspace_pose_to_traj(q_vec, move_time_est);
        traj_head = transitionTrajectories_.concat_trajs(traj_head, traj_tail); //concatenate trajectories  
        J1_ang = q_vec[0];
        //extraction with negative increments of J1 and negative increments of d8
        delta_J1 = (-M_PI / 2.0 - J1_ang) / 15.0; //-0.1;
        old_q_vec = q_vec;
        for (int i = 0; i < 15; i++) {
            old_q_vec = q_vec;
            J1_ang += delta_J1;
            q_vec[0] = J1_ang;
            //CHECK SIGN OF THIS!
            q_vec[6] = q_vec[6]+delta_J1; //counter-rotate flange to avoid part hitting uprights
            
            delta_d8 = R_OUTSTRETCHED * cos(J1_ang) * delta_J1;
            q_vec[7] += delta_d8;
            move_time_est = estimate_move_time(old_q_vec, q_vec) + 0.1;
            traj_tail = jspace_pose_to_traj(q_vec, move_time_est);
            traj_head = transitionTrajectories_.concat_trajs(traj_head, traj_tail); //concatenate trajectories   
        }
        old_q_vec = q_vec;

        //transitionTrajectories_.c_array_to_qvec(BIN1_DEEP_CRUISE_array,q_vec);
        //if (y_part> -0.35) y_part = -0.35;
        q_vec = bin5_deep_cruise_pose;
        d8_disp = y_part - R_OUTSTRETCHED - Y_BASE_WRT_WORLD_AT_D8_HOME;
        q_vec[0] = -1.57;
        q_vec[2] = 0.0; //freeze this joint
        q_vec[6] = old_q_vec[6]; // don't care about tool flange rot,  so don't wait for it
        q_vec[7] = old_q_vec[7]; //don't bother to  move sled further

        //q_vec[0]=1.57;
        move_time_est = estimate_move_time(old_q_vec, q_vec) + 0.5; //+0.5;     

        traj_tail = jspace_pose_to_traj(q_vec, move_time_est);
        traj_head = transitionTrajectories_.concat_trajs(traj_head, traj_tail); //concatenate trajectories             
        send_traj_goal(traj_head, CUSTOM_JSPACE_POSE);
        ROS_INFO("sent multipoint traj to withdraw arm from deep row of bin");
            if (rtn_state_ == bad_state_) {
                ROS_WARN("trying to recover from ABORT");
                if(try_recover_from_abort(q_vec)) {
                    ROS_INFO("recovery successful");
                }
                else {
                    ROS_WARN("recovery not successful within tolerance");
                }
            }          
    }
    else {
        ROS_WARN("extracting arm from  middle row of bin");
        //depart:
        traj_head = jspace_pose_to_traj(desired_approach_jspace_pose_, 1.0);
        //transitionTrajectories_.c_array_to_qvec(BIN1_DEEP_VIA_2,q_vec);
        q_vec = desired_approach_jspace_pose_;
        old_q_vec = q_vec;

        q_vec[1] = -1.35;
        q_vec[3] = 0.3;
        q_vec[5] = 1.4;
        move_time_est = estimate_move_time(old_q_vec, q_vec) + 0.5;
        traj_tail = jspace_pose_to_traj(q_vec, move_time_est);
        traj_head = transitionTrajectories_.concat_trajs(traj_head, traj_tail); //concatenate trajectories  
        J1_ang = q_vec[0];
        delta_J1 = (-M_PI / 2.0 - J1_ang) / 15.0; //-0.1; //negative increments of J1 and  negative increments of d8            
        //delta_J1 = 0.1;
        for (int i = 0; i < 15; i++) {
            old_q_vec = q_vec;
            J1_ang += delta_J1;
            q_vec[0] = J1_ang;
            q_vec[6] = q_vec[6]+delta_J1; //counter-rotate flange to avoid part hitting uprights            
            delta_d8 = R_OUTSTRETCHED * cos(J1_ang) * delta_J1;
            q_vec[7] += delta_d8;
            move_time_est = estimate_move_time(old_q_vec, q_vec) + 0.1;
            traj_tail = jspace_pose_to_traj(q_vec, move_time_est);
            traj_head = transitionTrajectories_.concat_trajs(traj_head, traj_tail); //concatenate trajectories   
        }
        old_q_vec = q_vec;
        q_vec[0] = -1.57;
        q_vec[2] = 0.0; //freeze this joint
        q_vec[6] = old_q_vec[6]; // don't care about tool flange rot,  so don't wait for it
        q_vec[7] = old_q_vec[7]; //don't bother to  move sled further        
        move_time_est = estimate_move_time(old_q_vec, q_vec) + 0.2;
        traj_tail = jspace_pose_to_traj(q_vec, move_time_est);
        traj_head = transitionTrajectories_.concat_trajs(traj_head, traj_tail); //concatenate trajectories             
        send_traj_goal(traj_head, CUSTOM_JSPACE_POSE);

        ROS_INFO("sent multipoint traj to withdraw arm from middle row of bin");
            if (rtn_state_ == bad_state_) {
                ROS_WARN("trying to recover from ABORT");
                if(try_recover_from_abort(q_vec)) {
                    ROS_INFO("recovery successful");
                }
                else {
                    ROS_WARN("recovery not successful within tolerance");
                }
            }          
    }
    is_attached_ = gripperInterface_.isGripperAttached();
    if (!is_attached_) {
        ROS_WARN("dropped  part!");
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::PART_DROPPED;
        return errorCode_;
    }   
    errorCode_ = robot_behavior_interface::RobotBehaviorResult::NO_ERROR; //return success
    if (!is_attached_) errorCode_ = robot_behavior_interface::RobotBehaviorResult::GRIPPER_FAULT;
    return errorCode_;
}

//won't use this

void KukaBehaviorActionServer::alt_fwd_qvec_from_rvrs_qvec(double part_y, Eigen::VectorXd &q_vec) {
    q_vec[0] = -q_vec[0];
    //q_vec[1] = -q_vec[1];
    q_vec[2] = 0.0;
    //q_vec[3] = -q_vec[3];
    //q_vec[4] = -q_vec[4]+M_PI; //check this for 3 wrist angs...
    //while (q_vec[4]>DH_q_max5) q_vec[4]-=2.0*M_PI;
    //q_vec[5] = q_vec[5];
    //q_vec[6] = -q_vec[6]; //+M_PI;
    //while (q_vec[6]>DH_q_max7) q_vec[6]-=2.0*M_PI;

    //sled position is special case--what is displacement relative to part_y; negate this offset relative to part_y
    double d8_bin1 = q_vec[7];
    //    d8 = y_part -Y_BASE_WRT_WORLD_AT_D8_HOME +fabs(dy_part_wrt_link0);
    double dsled = d8_bin1 - part_y + Y_BASE_WRT_WORLD_AT_D8_HOME;
    q_vec[7] = part_y - Y_BASE_WRT_WORLD_AT_D8_HOME - dsled;
}

void KukaBehaviorActionServer::goto_cruise_pose(Eigen::VectorXd desired_cruise_pose) {
    trajectory_msgs::JointTrajectory traj_head, traj_tail;
    //check if require righty/lefty flip:
    get_fresh_joint_states(); //refresh joint states
    double J1_start = joint_state_vec_[0];
    double J1_goal = desired_cruise_pose[0];
    ROS_INFO("J1_start, J1_goal = %f, %f", J1_start, J1_goal);
    Eigen::VectorXd q_via, q_old;
    q_via.resize(8);
    q_old.resize(8);
    q_via = joint_state_vec_;
    q_old = q_via;
    double move_time_est;
    if ((J1_goal - J1_start)< -2.0) { //require move from J1=+1.57 to J1 = -1.57
        ROS_WARN("flip J1 from +pi/2 to  -pi/2");
        //need to do a flip!
        //1.57, -1.35, 0, 0.5, 0, -1.2, 0, 0 //nom start

        //1.57, 0, 0, 0.5, 0, -1.2, 0, 0
        //0, 0, 0, 0.5, 0, -1.2, 0, 0
        //-1.57, 0, 0, 0.5, 0, -1.2, 0, 0
        //-1.57, -1.35, 0, 0.5, 0, -1.2, 0, 0
        q_via[0] = 1.57; //should be parallel to track
        q_via[1] = 0.0; //point arm up i.e. raise shoulder
        q_via[2] = 0.0; //freeze this joint
        q_via[3] = 0.5; // assure elbow is pretty straight
        q_via[7] = desired_cruise_pose[7]; //and head towarads desired sled position
        move_time_est = estimate_move_time(q_old, q_via) + 1.0;
        traj_head = jspace_pose_to_traj(q_via, move_time_est);
        q_old = q_via;
        q_via[0] = 0.0; //rotate J1 to face shelves
        move_time_est = estimate_move_time(q_old, q_via) + 0.5;
        traj_tail = jspace_pose_to_traj(q_via, move_time_est);
        traj_head = transitionTrajectories_.concat_trajs(traj_head, traj_tail);
        q_old = q_via;
        q_via[0] = -1.57; //rotate J1 to to complete righty/lefty rot
        move_time_est = estimate_move_time(q_old, q_via) + 0.5;
        traj_tail = jspace_pose_to_traj(q_via, move_time_est);
        traj_head = transitionTrajectories_.concat_trajs(traj_head, traj_tail);
        //bring arm back down:
        q_old = q_via;
        move_time_est = estimate_move_time(q_old, desired_cruise_pose) + 1.5;
        traj_tail = jspace_pose_to_traj(desired_cruise_pose, move_time_est);
        traj_head = transitionTrajectories_.concat_trajs(traj_head, traj_tail);

    } else if ((J1_goal - J1_start) > 2.0) { //require move from J1=-1.57 to J1 = +1.57
        ROS_WARN("flip J1 from -pi/2 to  +pi/2");

        //need to do a flip!
        //1.57, -1.35, 0, 0.5, 0, -1.2, 0, 0 //nom start

        //1.57, 0, 0, 0.5, 0, -1.2, 0, 0
        //0, 0, 0, 0.5, 0, -1.2, 0, 0
        //-1.57, 0, 0, 0.5, 0, -1.2, 0, 0
        //-1.57, -1.35, 0, 0.5, 0, -1.2, 0, 0
        q_via[0] = -1.57; //orient parallel to track       
        q_via[1] = 0.0; //point arm up
        q_via[2] = 0.0; //freeze this joint at  0
        q_via[3] = 0.5; // and elbow pretty straight
        q_via[7] = desired_cruise_pose[7]; //and head towarads desired sled position
        move_time_est = estimate_move_time(q_old, q_via) + 1.0;
        traj_head = jspace_pose_to_traj(q_via, move_time_est);
        q_old = q_via;
        q_via[0] = 0.0; //rotate J1 to face shelves
        move_time_est = estimate_move_time(q_old, q_via) + 0.5;
        traj_tail = jspace_pose_to_traj(q_via, move_time_est);
        traj_head = transitionTrajectories_.concat_trajs(traj_head, traj_tail);
        q_old = q_via;
        q_via[0] = +1.57; //rotate J1 to to complete righty/lefty rot
        move_time_est = estimate_move_time(q_old, q_via) + 0.5;
        traj_tail = jspace_pose_to_traj(q_via, move_time_est);
        traj_head = transitionTrajectories_.concat_trajs(traj_head, traj_tail);
        //bring arm back down:
        q_old = q_via;
        move_time_est = estimate_move_time(q_old, desired_cruise_pose) + 1.5;
        traj_tail = jspace_pose_to_traj(desired_cruise_pose, move_time_est);
        traj_head = transitionTrajectories_.concat_trajs(traj_head, traj_tail);

    } else {
        ROS_WARN("no flip required to get to cruise pose");
        move_time_est = estimate_move_time(joint_state_vec_, desired_cruise_pose) + 0.5;
        traj_head = jspace_pose_to_traj(desired_cruise_pose, move_time_est);
    }

    //for all cases, execute trajectory and eval convergence:    
    send_traj_goal(traj_head, CUSTOM_JSPACE_POSE);
    move_time_est = 1.0;

    //keep checking for convergence:
    int ntries = 0;
    while ((move_time_est > 0.06)&&(ntries < 10)) {
        ntries++;
        get_fresh_joint_states(); //refresh joint states
        move_time_est = estimate_move_time(joint_state_vec_, desired_cruise_pose) + 0.05;
        ROS_INFO("settling: move_time_est = %f", move_time_est);
        traj_head = jspace_pose_to_traj(desired_cruise_pose, move_time_est);
        send_traj_goal(traj_head, CUSTOM_JSPACE_POSE);
        ros::Duration(0.1).sleep();
    }

}

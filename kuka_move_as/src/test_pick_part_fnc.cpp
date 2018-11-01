
// use "goal", but only need to populate the "sourcePart" component
//replaced "deep"  and "near" with wrist-flip observation
const double X_DEEP = -0.83;
//see what happens w/ middle strategy on front row:
const double X_NEAR = -0.6; //-0.7;  //-0.73 is nom midpoint; try near up to -0.7

unsigned short int KukaBehaviorActionServer::test_pick_part_from_bin(const robot_behavior_interface::RobotBehaviorGoalConstPtr &goal) {
    int ans;
    inventory_msgs::Part part = goal->sourcePart;
    ROS_INFO("The part is %s; it should be fetched from location code %s ", part.name.c_str(),
            placeFinder_[part.location].c_str());
    ROS_INFO("part info: ");
    ROS_INFO_STREAM(part);
    trajectory_msgs::JointTrajectory traj_head,traj_tail; 
    Eigen::VectorXd q_vec,old_q_vec;
    q_vec.resize(NDOF);
    old_q_vec.resize(NDOF);
    double move_time_est;
    double J1_ang, d8_disp;
    double delta_d8,delta_J1;
    bool wrist_flip = false;

    //COMPUTE KEY POSES FOR MANIPULATION HERE:
    errorCode_ = compute_bin_pickup_key_poses(part);   
    if (errorCode_ != robot_behavior_interface::RobotBehaviorResult::NO_ERROR) {
        return errorCode_;
    }
    if (pickup_jspace_pose_[5]<0.0) wrist_flip=true; //use this for "deep" grasp solns

    // different strategy if "deep", i.e. part is in back row;
        bool is_deep = false;
        bool is_near = true;
        bool is_middle = false;
        double x_part = part.pose.pose.position.x;
        double y_part = part.pose.pose.position.y;
        if (x_part < X_DEEP) { is_deep = true; is_near=false; }
        else if (x_part > X_NEAR) { is_deep = false; is_near = true; }
        else { is_deep=false; is_near=false; is_middle=true; }
        //DO APPROACH:
        get_fresh_joint_states();
        //if (is_deep) {
        if (wrist_flip) {            
            ROS_INFO("back-row; computing insertion motion");
            //place the start here, at this cruise pose:
            transitionTrajectories_.c_array_to_qvec(BIN1_DEEP_CRUISE_array,q_vec);
            //need to assure elbow clearance
            get_fresh_joint_states(); // update joint_state_vec_
            if (desired_approach_jspace_pose_[1]> -1.34) desired_approach_jspace_pose_[1]= -1.34;
            //XXX FIX ME!
            //if (y_part> -0.35) y_part = -0.35; //GENERALIZE THIS TO OTHER BINS

            d8_disp = y_part + R_OUTSTRETCHED - Y_BASE_WRT_WORLD_AT_D8_HOME;
            ROS_INFO("compute d8 = %f",d8_disp);
            q_vec[7] = d8_disp;
            //q_vec[7] = d8_disp;  //correct the rail displacement, per computation
            J1_ang = q_vec[0];
            delta_J1 = (M_PI/2 - desired_approach_jspace_pose_[0])/10.0;
            goto_cruise_pose(q_vec);

            //compute move time to get to this pose:
            /*
            move_time_est = estimate_move_time(joint_state_vec_,q_vec)+1;   
            ROS_INFO("estimated move time to cruise start = %f",move_time_est);
            traj_head = jspace_pose_to_traj(q_vec,move_time_est); 
            send_traj_goal(traj_head,CUSTOM_JSPACE_POSE); //send this now--may need to confirm it is successful
            ros::spinOnce(); //update joint state vec
            //check that robot is converged:
            //IF THIS IS SUCCESSFUL,  PROPAGATE IT TO OTHER CASES
            move_time_est=1.0;*/
            while (move_time_est>0.02) {
                get_fresh_joint_states(); //refresh joint states
                move_time_est = estimate_move_time(joint_state_vec_,q_vec);
                ROS_INFO("settling: move_time_est = %f",move_time_est);
                traj_head = jspace_pose_to_traj(q_vec,move_time_est); 
                send_traj_goal(traj_head,CUSTOM_JSPACE_POSE);    
                ros::Duration(0.2).sleep();
            }
            ROS_INFO("cruise pose has settled; start deep reach into bin");
            //reset the head of the trajectory:
            move_time_est = estimate_move_time(joint_state_vec_,q_vec)+0.1;   
            traj_head = jspace_pose_to_traj(q_vec,move_time_est);

            //move forward, computing coordinated J1/d8 for pure x motion (const y)       
            for (int i=0;i<10;i++) {
                old_q_vec = q_vec;
                J1_ang -= delta_J1;
                delta_d8 = R_OUTSTRETCHED*cos(J1_ang)*delta_J1;
                d8_disp-= delta_d8;     
                q_vec[0] = J1_ang;
                q_vec[7] = d8_disp;
                move_time_est = estimate_move_time(old_q_vec,q_vec)+0.08;     
                traj_tail = jspace_pose_to_traj(q_vec,move_time_est); 
                traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories 
            }      
            //finish up at approach pose:
            move_time_est = estimate_move_time(q_vec,desired_approach_jspace_pose_)+0.2;
            traj_tail = jspace_pose_to_traj(desired_approach_jspace_pose_,move_time_est);
            traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories             
            
            send_traj_goal(traj_head,CUSTOM_JSPACE_POSE);
            //ROS_INFO("pausing at approach pose: ");
            //ros::Duration(2.0).sleep();
            ROS_INFO("sent multipoint traj to approach pose for part in back row");
        }

        else {
            ROS_WARN("acquiring part from front or middle row of bin...");

            transitionTrajectories_.c_array_to_qvec(BIN1_CENTER_ROW_CRUISE_array,q_vec);
            if (desired_approach_jspace_pose_[1]> -1.34) desired_approach_jspace_pose_[1]= -1.34;
            // FIX ME!!  reference y-value to current bin
            //if (y_part> -0.35) y_part = -0.35;
            d8_disp = y_part + R_OUTSTRETCHED - Y_BASE_WRT_WORLD_AT_D8_HOME;
            q_vec[7] = d8_disp;
            //q_vec[7] = d8_disp;  //correct the rail displacement, per computation
            J1_ang = q_vec[0];
            goto_cruise_pose(q_vec); //approach cruise pose; do flip, if necessary
            delta_J1 = (M_PI/2 - desired_approach_jspace_pose_[0])/10.0;
            //compute move time to get to this pose:
            /*
            move_time_est = estimate_move_time(joint_state_vec_,q_vec)+0.5;            
            traj_head = jspace_pose_to_traj(q_vec,move_time_est); 
             * */
            //send_traj_goal(traj_head,CUSTOM_JSPACE_POSE);
            //cout<<"enter  1: ";
            //cin>>ans;
            //should already be at cruise pose, so initial move time should be fast...just
            // a convenient starting point for multi-point trajectory
            move_time_est = estimate_move_time(joint_state_vec_,q_vec)+0.1;            
            traj_head = jspace_pose_to_traj(q_vec,move_time_est); 
            //move forward, computing coordinated J1/d8 for pure x motion (const y)       
            for (int i=0;i<10;i++) {
                old_q_vec = q_vec;
                J1_ang -= delta_J1;
                delta_d8 = R_OUTSTRETCHED*cos(J1_ang)*delta_J1;
                d8_disp-= delta_d8;     
                q_vec[0] = J1_ang;
                q_vec[7] = d8_disp;
                move_time_est = estimate_move_time(old_q_vec,q_vec)+0.1;     
                traj_tail = jspace_pose_to_traj(q_vec,move_time_est); 
                traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories 
            }      
            //finish up at approach pose:
            move_time_est = estimate_move_time(q_vec,desired_approach_jspace_pose_)+0.5;
            traj_tail = jspace_pose_to_traj(desired_approach_jspace_pose_,move_time_est);
            traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories             
                               
                    
            send_traj_goal(traj_head,CUSTOM_JSPACE_POSE);
            ROS_INFO("sent multipoint traj to approach pose for part in middle row");            
        }
    
    ROS_WARN(" DO PICKUP STEPS HERE...");
    
    ROS_INFO("descending to grasp pose, ");

    gripperInterface_.grab(); //do this early, so grasp can occur at first contact
    is_attached_ =  false;
    ROS_INFO("descending to grasp pose, ");

    move_into_grasp(pickup_jspace_pose_, 1.5); //provide target pose
    ROS_INFO("at computed grasp pose;checking for gripper attachment ");    
    is_attached_ = gripperInterface_.waitForGripperAttach(2.0); //wait for grasp for a bit


    if (!is_attached_) move_into_grasp(MOVE_INTO_GRASP_TIME);


    //EXTRACT ARM FROM BIN:
    if (wrist_flip) {  
    //else if (is_deep) {
        ROS_WARN("extracting arm from back row of bin");
            //transitionTrajectories_.c_array_to_qvec(BIN1_DEEP_VIA_2,q_vec);
            q_vec = desired_approach_jspace_pose_;
            get_fresh_joint_states(); // update joint_state_vec_
            q_vec[6] = joint_state_vec_[6]; //leave the tool flange rotation alone
            traj_head = jspace_pose_to_traj(q_vec,0.8);
            old_q_vec = q_vec;
            // NOT SO GOOD--elbow hits; need elbow straighter
            //outstretched arm:
            //[0.172, -1.3,        0,  0.3,   -0.043,   -1.4,   2.945,  -1.338
            q_vec[1] = -1.35;
            q_vec[3] = 0.3;
            q_vec[5] = -1.4;
            move_time_est = estimate_move_time(old_q_vec,q_vec)+0.8;     

            traj_tail = jspace_pose_to_traj(q_vec,move_time_est); 
            traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories  
            J1_ang= q_vec[0];
            delta_J1 = (1.57- q_vec[0])/15.0;//0.1;
            old_q_vec = q_vec;
            for (int i=0;i<15;i++) {
               old_q_vec = q_vec;
               J1_ang += delta_J1;
               q_vec[0]=J1_ang;
               q_vec[6] = q_vec[6]+delta_J1; //counter-rotate flange to avoid part hitting uprights
               delta_d8 = R_OUTSTRETCHED*cos(J1_ang)*delta_J1;
               q_vec[7]+= delta_d8;
               move_time_est = estimate_move_time(old_q_vec,q_vec)+0.1;     
               traj_tail = jspace_pose_to_traj(q_vec,move_time_est); 
               traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories   
            }
            old_q_vec = q_vec;
            
            transitionTrajectories_.c_array_to_qvec(BIN1_DEEP_CRUISE_array,q_vec);
            q_vec[0] = M_PI/2.0;
            //if (y_part> -0.35) y_part = -0.35;
            //d8_disp = y_part + R_OUTSTRETCHED - Y_BASE_WRT_WORLD_AT_D8_HOME;
            q_vec[6] = old_q_vec[6]; //  don't care about flange rotation,  so don't wait for it
            q_vec[7] = old_q_vec[7];
            
            //q_vec[0]=1.57;
            move_time_est = estimate_move_time(old_q_vec,q_vec)+0.5; //+0.5;     
            
            traj_tail = jspace_pose_to_traj(q_vec,move_time_est); 
            traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories             
            send_traj_goal(traj_head,CUSTOM_JSPACE_POSE);
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
            q_vec = desired_approach_jspace_pose_;
            get_fresh_joint_states(); // update joint_state_vec_
            q_vec[6] = joint_state_vec_[6]; //leave the tool flange rotation alone
            traj_head = jspace_pose_to_traj(q_vec,1.0);
            old_q_vec = q_vec;

            q_vec[1] = -1.35;
            q_vec[3] = 0.3;
            q_vec[5] = 1.4;
            move_time_est = estimate_move_time(old_q_vec,q_vec)+0.8;
            traj_tail = jspace_pose_to_traj(q_vec,move_time_est); 
            traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories  
            J1_ang= q_vec[0];
            //delta_J1 = 0.1;
            delta_J1 = (1.57- q_vec[0])/15.0;
            for (int i=0;i<15;i++) {
               old_q_vec = q_vec; 
               J1_ang += delta_J1;
               q_vec[0]=J1_ang;
               q_vec[6] = q_vec[6]+delta_J1; //counter-rotate flange to avoid part hitting uprights               
               delta_d8 = R_OUTSTRETCHED*cos(J1_ang)*delta_J1;
               q_vec[7]+= delta_d8;
               move_time_est = estimate_move_time(old_q_vec,q_vec)+0.1;
               traj_tail = jspace_pose_to_traj(q_vec,move_time_est); 
               traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories   
            }
            old_q_vec = q_vec;
            transitionTrajectories_.c_array_to_qvec(BIN1_DEEP_CRUISE_array,q_vec);
            q_vec[0]=1.57;
            q_vec[6] = old_q_vec[6]; //  don't care about flange rotation,  so don't wait for it           
            q_vec[7] = old_q_vec[7];
            
            move_time_est = estimate_move_time(old_q_vec,q_vec)+0.5;
            traj_tail = jspace_pose_to_traj(q_vec,move_time_est); 
            traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories             
            send_traj_goal(traj_head,CUSTOM_JSPACE_POSE);
           
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
        if(!is_attached_) errorCode_ = robot_behavior_interface::RobotBehaviorResult::GRIPPER_FAULT;
        return errorCode_;
}



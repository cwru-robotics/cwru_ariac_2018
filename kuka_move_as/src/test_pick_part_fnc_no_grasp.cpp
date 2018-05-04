
// use "goal", but only need to populate the "sourcePart" component
const double X_DEEP = -0.83;
//see what happens w/ middle strategy on front row:
const double X_NEAR = -0.6; //-0.7;  //-0.73 is nom midpoint; try near up to -0.7

unsigned short int KukaBehaviorActionServer::test_pick_part_from_bin(const kuka_move_as::RobotBehaviorGoalConstPtr &goal) {
    //unsigned short int errorCode = kuka_move_as::RobotBehaviorResult::NO_ERROR; //return this if ultimately successful
    //trajectory_msgs::JointTrajectory transition_traj;
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

        errorCode_ = compute_bin_pickup_key_poses(part);   
    if (errorCode_ != kuka_move_as::RobotBehaviorResult::NO_ERROR) {
        
        return errorCode_;
    }

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
        ros::spinOnce();
        if (is_deep) {
            ROS_INFO("back-row; commputing insertion motion");
            //place the start here, at this cruise pose:
            transitionTrajectories_.c_array_to_qvec(BIN1_DEEP_CRUISE_array,q_vec);
            //need to assure elbow clearance
            ros::spinOnce(); // want to update joint states
            if (desired_approach_jspace_pose_[1]> -1.34) desired_approach_jspace_pose_[1]= -1.34;
            if (y_part> -0.35) y_part = -0.35;
            d8_disp = y_part + R_OUTSTRETCHED - Y_BASE_WRT_WORLD_AT_D8_HOME;
            ROS_INFO("compute d8 = %f",d8_disp);
            q_vec[7] = d8_disp;
            //q_vec[7] = d8_disp;  //correct the rail displacement, per computation
            J1_ang = q_vec[0];
            delta_J1 = (M_PI/2 - desired_approach_jspace_pose_[0])/10.0;
            //compute move time to get to this pose:
            move_time_est = estimate_move_time(joint_state_vec_,q_vec)+1;   
            ROS_INFO("estimated move time to cruise start = %f",move_time_est);
            traj_head = jspace_pose_to_traj(q_vec,move_time_est); 
            send_traj_goal(traj_head,CUSTOM_JSPACE_POSE); //send this now--may need to confirm it is successful
            ros::spinOnce(); //update joint state vec
            //check that robot is converged:
            //IF THIS IS SUCCESSFUL,  PROPAGATE IT TO OTHER CASES
            move_time_est=1.0;
            while (move_time_est>0.02) {
                ros::spinOnce(); //refresh joint states
                move_time_est = estimate_move_time(joint_state_vec_,q_vec);
                ROS_INFO("settling: move_time_est = %f",move_time_est);
                traj_head = jspace_pose_to_traj(q_vec,move_time_est); 
                send_traj_goal(traj_head,CUSTOM_JSPACE_POSE);               
            }
            ROS_INFO("cruise pose has settled; start deep reach into bin");
            //reset the head of the trajectory:
            move_time_est = estimate_move_time(joint_state_vec_,q_vec)+0.1;   
            traj_head = jspace_pose_to_traj(q_vec,move_time_est);
            /*
            move_time_est=1.0;
            while (move_time_est>0.01) {
               move_time_est = estimate_move_time(joint_state_vec_,q_vec)+1;   
               ROS_INFO("estimated move time to cruise start = %f",move_time_est);
               traj_head = jspace_pose_to_traj(q_vec,move_time_est);     
            }
             * */
            //cout<<"enter  1: ";
            //cin>>ans;

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
        else if (is_near) {
            ROS_WARN("reaching for part in front row of bin");
            ros::spinOnce();
            transitionTrajectories_.c_array_to_qvec(BIN1_NEAR_CRUISE_array,q_vec);
            move_time_est = estimate_move_time(joint_state_vec_,q_vec)+0.5;            
            traj_head = jspace_pose_to_traj(q_vec,move_time_est);            
            traj_tail = jspace_pose_to_traj(desired_approach_jspace_pose_,1.0);
            traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories  
            send_traj_goal(traj_head,CUSTOM_JSPACE_POSE);
            ROS_INFO("sent multipoint traj to approach pose for part in front row");
        }
        else {
            ROS_WARN("part in middle row...not ready");
            //BIN1_CENTER_ROW_CRUISE_array
            /*
            old_q_vec = q_vec;
            transitionTrajectories_.c_array_to_qvec(BIN1_CENTER_ROW_CRUISE_array,q_vec);
            move_time_est = estimate_move_time(joint_state_vec_,q_vec)+0.5;            
            traj_head = jspace_pose_to_traj(q_vec,move_time_est);   
            old_q_vec = q_vec;
            transitionTrajectories_.c_array_to_qvec(BIN1_CENTER_ROW_VIA_1,q_vec);
            move_time_est = estimate_move_time(old_q_vec,q_vec)+0.5;             
            traj_head = jspace_pose_to_traj(q_vec,move_time_est);             
            //traj_tail = jspace_pose_to_traj(q_vec,1.5);    
            traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories  
            old_q_vec = q_vec;
            transitionTrajectories_.c_array_to_qvec(BIN1_CENTER_ROW_VIA_2,q_vec);
            move_time_est = estimate_move_time(old_q_vec,q_vec)+0.5;             
            traj_head = jspace_pose_to_traj(q_vec,move_time_est);             
            //traj_tail = jspace_pose_to_traj(q_vec,1.5);    
            traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories  
            traj_tail = jspace_pose_to_traj(desired_approach_jspace_pose_,1.0);
            traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories  
            
            xxx*/
            //transitionTrajectories_.c_array_to_qvec(BIN1_DEEP_CRUISE_array,q_vec);
            transitionTrajectories_.c_array_to_qvec(BIN1_CENTER_ROW_CRUISE_array,q_vec);
            if (desired_approach_jspace_pose_[1]> -1.34) desired_approach_jspace_pose_[1]= -1.34;

            if (y_part> -0.35) y_part = -0.35;
            d8_disp = y_part + R_OUTSTRETCHED - Y_BASE_WRT_WORLD_AT_D8_HOME;
            q_vec[7] = d8_disp;
            //q_vec[7] = d8_disp;  //correct the rail displacement, per computation
            J1_ang = q_vec[0];
            delta_J1 = (M_PI/2 - desired_approach_jspace_pose_[0])/10.0;
            //compute move time to get to this pose:
            move_time_est = estimate_move_time(joint_state_vec_,q_vec)+0.5;            
            traj_head = jspace_pose_to_traj(q_vec,move_time_est); 
            //send_traj_goal(traj_head,CUSTOM_JSPACE_POSE);
            //cout<<"enter  1: ";
            //cin>>ans;

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
 
    move_into_grasp(pickup_jspace_pose_, 1.5); //provide target pose
    ROS_INFO("at computed grasp pose ");    

     //combine these  moves: approach_pickup_jspace_pose_, computed_bin_escape_jspace_pose_,
    //current_bin_cruise_pose_ w/ frozen wrist, current_bin_cruise_pose_
    traj_head = jspace_pose_to_traj(desired_depart_jspace_pose_,1.0);

    ROS_INFO(" moving to depart pose ");//approach_pickup_jspace_pose_
    //cin>>ans;
    
    //ROS_INFO("moving to hover_jspace_pose_ ");
    //freeze wrist:
    //for (int i=4;i<6;i++) current_hover_pose_[i] = desired_depart_jspace_pose_[i];
    //traj_tail = jspace_pose_to_traj(current_hover_pose_,1.0);    
    //traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories
    
    //EXTRACT ARM FROM BIN:
    ROS_INFO("extracting arm");
    if (is_near) {
        ROS_WARN("extracting arm from front row of bin");
            traj_head = jspace_pose_to_traj(desired_approach_jspace_pose_,0.8); //move from grasp to depart  pose
            //J1= 0.75,  d8 = -0.97
            //J1= 0.9,   d8 = -0.9
            //J1= 1.0,   d8 = -0.85
            //J1= 1.1,   d8 = -0.8
            q_vec = desired_approach_jspace_pose_;
            q_vec[0]= q_vec[0]+0.1;
            q_vec[7]=q_vec[7]+0.05;            
            traj_tail = jspace_pose_to_traj(q_vec,0.2);  //swing out to cruise pose             
            traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories  
            q_vec[0]= q_vec[0]+0.1;
            q_vec[7]=q_vec[7]+0.05;            
            traj_tail = jspace_pose_to_traj(q_vec,0.1);  //swing out to cruise pose             
            traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories  
            q_vec[0]= q_vec[0]+0.1;
            q_vec[7]=q_vec[7]+0.05;            
            traj_tail = jspace_pose_to_traj(q_vec,0.1);  //swing out to cruise pose             
            traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories              
            old_q_vec = q_vec;
            transitionTrajectories_.c_array_to_qvec(BIN1_NEAR_CRUISE_array,q_vec);
            traj_tail = jspace_pose_to_traj(q_vec,1);  //swing out to cruise pose             
            traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories              
            
            
            send_traj_goal(traj_head,CUSTOM_JSPACE_POSE);
            ROS_INFO("sent multipoint traj to extract arm from front row of bin");
        }    
    else if (is_deep) {
        ROS_WARN("extracting arm from back row of bin");
            traj_head = jspace_pose_to_traj(desired_approach_jspace_pose_,0.8);
            //transitionTrajectories_.c_array_to_qvec(BIN1_DEEP_VIA_2,q_vec);
            q_vec = desired_approach_jspace_pose_;

            // NOT SO GOOD--elbow hits; need elbow straighter
            //outstretched arm:
            //[0.172, -1.3,        0,  0.3,   -0.043,   -1.4,   2.945,  -1.338
            q_vec[1] = -1.35;
            q_vec[3] = 0.3;
            q_vec[5] = -1.4;
            move_time_est = estimate_move_time(desired_approach_jspace_pose_,q_vec)+0.5;     

            traj_tail = jspace_pose_to_traj(q_vec,move_time_est); 
            traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories  
            J1_ang= q_vec[0];
            delta_J1 = 0.1;
            old_q_vec = q_vec;
            for (int i=0;i<9;i++) {
               old_q_vec = q_vec;
               J1_ang += delta_J1;
               q_vec[0]=J1_ang;
               delta_d8 = R_OUTSTRETCHED*cos(J1_ang)*delta_J1;
               q_vec[7]+= delta_d8;
               move_time_est = estimate_move_time(old_q_vec,q_vec)+0.1;     
               traj_tail = jspace_pose_to_traj(q_vec,move_time_est); 
               traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories   
            }
            old_q_vec = q_vec;
            
            transitionTrajectories_.c_array_to_qvec(BIN1_DEEP_CRUISE_array,q_vec);
            if (y_part> -0.35) y_part = -0.35;
            d8_disp = y_part + R_OUTSTRETCHED - Y_BASE_WRT_WORLD_AT_D8_HOME;
            q_vec[7] = d8_disp;
            
            //q_vec[0]=1.57;
            move_time_est = estimate_move_time(old_q_vec,q_vec)+0.5; //+0.5;     
            
            traj_tail = jspace_pose_to_traj(q_vec,move_time_est); 
            traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories             
            send_traj_goal(traj_head,CUSTOM_JSPACE_POSE);
            ROS_INFO("sent multipoint traj to withdraw arm from deep row of bin");
        }

    else {
        ROS_WARN("extracting arm from  middle row of bin");
          //depart:
          traj_head = jspace_pose_to_traj(desired_approach_jspace_pose_,1.0);
            //transitionTrajectories_.c_array_to_qvec(BIN1_DEEP_VIA_2,q_vec);
            q_vec = desired_approach_jspace_pose_;
            // NOT SO GOOD--elbow hits; need elbow straighter
            //outstretched arm:
            //[0.172, -1.3,        0,  0.3,   -0.043,   -1.4,   2.945,  -1.338
            //1.57, -1.35, 0, 0.3, 2.93, 0.78, 0.387, -0.8 =  cruise array
            old_q_vec = q_vec;

            q_vec[1] = -1.35;
            q_vec[3] = 0.3;
            q_vec[5] = 1.4;
            move_time_est = estimate_move_time(old_q_vec,q_vec)+0.5;
            traj_tail = jspace_pose_to_traj(q_vec,move_time_est); 
            traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories  
            J1_ang= q_vec[0];
            delta_J1 = 0.1;
            for (int i=0;i<9;i++) {
               old_q_vec = q_vec; 
               J1_ang += delta_J1;
               q_vec[0]=J1_ang;
               delta_d8 = R_OUTSTRETCHED*cos(J1_ang)*delta_J1;
               q_vec[7]+= delta_d8;
               move_time_est = estimate_move_time(old_q_vec,q_vec)+0.08;
               traj_tail = jspace_pose_to_traj(q_vec,move_time_est); 
               traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories   
            }
            old_q_vec = q_vec;
            q_vec[0]=1.57;
            move_time_est = estimate_move_time(old_q_vec,q_vec)+0.2;
            traj_tail = jspace_pose_to_traj(q_vec,move_time_est); 
            traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories             
            send_traj_goal(traj_head,CUSTOM_JSPACE_POSE);
            
            /*
            traj_head = jspace_pose_to_traj(desired_approach_jspace_pose_,1.0);
            transitionTrajectories_.c_array_to_qvec(BIN1_CENTER_ROW_VIA_2,q_vec);
            traj_tail = jspace_pose_to_traj(q_vec,1.5); 
            traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories  
            old_q_vec = q_vec;
            transitionTrajectories_.c_array_to_qvec(BIN1_CENTER_ROW_VIA_1,q_vec);
            traj_tail = jspace_pose_to_traj(q_vec,1.5);  
            traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories  
            old_q_vec = q_vec;
            transitionTrajectories_.c_array_to_qvec(BIN1_CENTER_ROW_CRUISE_array,q_vec);
            traj_tail = jspace_pose_to_traj(q_vec,3.0);
            traj_head = transitionTrajectories_.concat_trajs(traj_head,traj_tail); //concatenate trajectories  
             */
            //send_traj_goal(traj_head,CUSTOM_JSPACE_POSE);
            ROS_INFO("sent multipoint traj to withdraw arm from middle row of bin");        
    }
        
        errorCode_ = kuka_move_as::RobotBehaviorResult::NO_ERROR; //return success
        return errorCode_;
}

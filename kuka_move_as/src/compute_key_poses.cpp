//fncs to help compute key jspace poses


/* NEED A FNC FOR THIS:
 *****formula for J1 and d8 as a func of part (x,y) in bin****
 * for bins approached with "reverse" IK soln (presently, bins 1-4):
given bin part location (x,y) w/rt world:
RADIUS*sin(pi/2-J1) = fabs(dx) 
--> fabs(dx)/RADIUS = sin(pi/2-theta_J1) 
    theta_J1 = pi/2 - asin(fabs(dx)/RADIUS) 

AND: dy_part_wrt_link0 = RADIUS*cos(1.57-theta_J1)
d8 = y_part_wrt_world - fabs(dy_part_wrt_link0)
 * 
 * 
 *
 */


//j1 is limited to +/-170 deg.  Therefore, choose soln J1 close to zero
//how to choose d8?
// for "far" parts, line up d8 with part_y
// for "near" parts, may need to approach at an angle...
// but want to approach part head-on as close as possible, to avoid interference with shelf upright
// want to keep J2< -1.35 to avoid hitting top crossbar;
//
// fnc compute_bin_hover_from_xy() computes a nominal approach, from which d8 follows;
// change this to bin5 approach by subtracting 2*radius and negating J1
// use: fwd_qvec_from_rvrs_qvec()


unsigned short int KukaBehaviorActionServer::compute_bin_pickup_key_poses(inventory_msgs::Part part) {
    errorCode_ = robot_behavior_interface::RobotBehaviorResult::NO_ERROR;    

    
    transitionTrajectories_.get_hover_pose(part.location,current_hover_pose_,current_bin_hover_pose_code_);
    
    current_bin_hover_pose_ = current_hover_pose_; //synonym
    
    computed_jspace_approach_.resize(8);
    
    //back to useful stuff:
    geometry_msgs::PoseStamped part_pose_wrt_world = part.pose;
    Eigen::Affine3d affine_part_wrt_world = xformUtils_.transformPoseToEigenAffine3d(part_pose_wrt_world);
    Eigen::Vector3d O_part_wrt_world = affine_part_wrt_world.translation();
    ROS_INFO_STREAM("part origin w/rt world: "<<O_part_wrt_world.transpose()<<endl);
    double part_x = O_part_wrt_world[0];
    double part_y = O_part_wrt_world[1];
    //adjust pickup x and y, as necessary, to avoid hitting bin or overreach
    if(!bin_xy_is_reachable(part.location,part_x, part_y))
     {
       O_part_wrt_world[0] = part_x;
       O_part_wrt_world[1]= part_y;   
       affine_part_wrt_world.translation() = O_part_wrt_world;
       //re-specify pick-up pose attempt:
       part.pose.pose.position.x = part_x;
       part.pose.pose.position.y = part_y;
    }

    if(!compute_bin_hover_from_xy(part_x,part_y, computed_jspace_approach_)) {
        ROS_WARN("could not compute valid approach to bin--something wrong");
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::UNREACHABLE;
        return errorCode_;
    }
        ROS_INFO_STREAM("computed jspace approach: "<<computed_jspace_approach_.transpose()<<endl);

    //computed_jspace_approach_ contains pose estimate for  IKs
    //compute the IK for the desired pickup pose: pickup_jspace_pose_
    //first, get the equivalent desired affine of the vacuum gripper w/rt base_link;
    //need to provide the Part info and the rail displacement
    //Eigen::Affine3d RobotMoveActionServer::affine_vacuum_pickup_pose_wrt_base_link(Part part, double q_rail)
    affine_vacuum_pickup_pose_wrt_base_link_ = affine_vacuum_pickup_pose_wrt_base_link(part,
            computed_jspace_approach_[7]);
    
    //NEED THE FOLLOWING VALUE for pickup_jspace_pose_
    //provide desired gripper pose w/rt base_link, and choose soln closest to some reference jspace pose, e.g. hover pose
    //if (!get_pickup_IK(cart_grasp_pose_wrt_base_link,computed_jspace_approach_,&q_vec_soln);
    if (!compute_pickup_dropoff_IK(affine_vacuum_pickup_pose_wrt_base_link_, computed_jspace_approach_, pickup_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for pickup pose!");
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::UNREACHABLE;
        return errorCode_;
    }

    ROS_INFO_STREAM("pickup_jspace_pose_: " << pickup_jspace_pose_.transpose());

    //compute approach_pickup_jspace_pose_:  for approximate Cartesian descent and depart
    //compute_approach_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd computed_jspace_approach_,double approach_dist,Eigen::VectorXd &q_vec_soln);
    if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, pickup_jspace_pose_, approach_dist_,
            desired_approach_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for pickup approach pose!");
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::UNREACHABLE;
        return errorCode_;
    }
    if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, pickup_jspace_pose_, depart_dist_,
            desired_depart_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for depart pose!");
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::UNREACHABLE;
        return errorCode_;
    }    
    
 //   
    //desired_approach_depart_pose_ = approach_pickup_jspace_pose_;
    ROS_INFO_STREAM("desired_approach_jspace_pose_: " << desired_approach_jspace_pose_.transpose());
    //modify computed_jspace_approach_ to use same wrist orientation as soln
    /*
    for (int i=4;i<7;i++) { current_hover_pose_[i] = desired_approach_jspace_pose_[i];
                            computed_jspace_approach_[i] = desired_approach_jspace_pose_[i];
                            computed_bin_escape_jspace_pose_[i] = desired_approach_jspace_pose_[i]; }    
    */
    if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, pickup_jspace_pose_, deep_grasp_dist_,
            pickup_deeper_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for deep pickup approach pose");
        ROS_WARN("re-using grasp pose; not returning an error");
        pickup_deeper_jspace_pose_ = pickup_jspace_pose_;
        //errorCode_ = robot_behavior_interface::RobotBehaviorResult::UNREACHABLE;
        //return errorCode_;
    }
    ROS_INFO_STREAM("pickup_deeper_jspace_pose_: " << pickup_deeper_jspace_pose_.transpose());  
    
    errorCode_ = robot_behavior_interface::RobotBehaviorResult::NO_ERROR;   
    
    //APPLY SYMMETRY FOR BIN5 CASE:
    //DO THIS IN BIN_PICK CODE INSTEAD...
    // fix: computed_jspace_approach_, computed_bin_escape_jspace_pose_, pickup_jspace_pose_, desired_approach_jspace_pose_, pickup_deeper_jspace_pose_
    /*
    if (part.location==Part::BIN5||part.location==Part::BIN4) {
         fwd_qvec_from_rvrs_qvec(part_y, computed_jspace_approach_);
         fwd_qvec_from_rvrs_qvec(part_y, computed_bin_cruise_jspace_pose_);
         fwd_qvec_from_rvrs_qvec(part_y, pickup_jspace_pose_);
         fwd_qvec_from_rvrs_qvec(part_y, desired_approach_jspace_pose_);
         fwd_qvec_from_rvrs_qvec(part_y, desired_depart_jspace_pose_);         
         fwd_qvec_from_rvrs_qvec(part_y, pickup_deeper_jspace_pose_);
         
         //the following are obsolete:
         //try hard coding the bin5 escape pose:
         transitionTrajectories_.c_array_to_qvec(BIN5_ESCAPE_array,computed_bin_escape_jspace_pose_);

         computed_bin_cruise_jspace_pose_= computed_bin_escape_jspace_pose_;
         //computed_bin_escape_jspace_pose_[7]+=0.1; //fix??         
         
         computed_bin_cruise_jspace_pose_[0] = 1.5707; //just rotate J1
         computed_bin_cruise_jspace_pose_[7] -=0.3; //move rail together with arm swing
         //coerce the wrist to agree w/ IK poses
         for (int i=4;i<7;i++) { current_hover_pose_[i] = desired_depart_jspace_pose_[i];
                            computed_jspace_approach_[i] = desired_depart_jspace_pose_[i];
                            computed_bin_escape_jspace_pose_[i] = desired_depart_jspace_pose_[i]; }  


         
    }  */

    ROS_INFO("SUCCESSFUL COMPUTATION OF KEY POSES FOR BIN PICKUP");
    return errorCode_;
        
}

//having solved for bin1 IK key poses, infer corresponding non-flipped poses applicable to bin5:
    //APPLY SYMMETRY FOR BIN5 CASE:
    // fix: computed_jspace_approach_, computed_bin_escape_jspace_pose_, pickup_jspace_pose_, approach_pickup_jspace_pose_, pickup_deeper_jspace_pose_
  //get corresponding "fwd" soln from reverse soln; includes sled offset in opposite direction
void KukaBehaviorActionServer::fwd_qvec_from_rvrs_qvec(double part_y, Eigen::VectorXd &q_vec) {
    q_vec[0] = M_PI - q_vec[0];
    q_vec[1] = -q_vec[1];
    q_vec[2] = 0.0;
    q_vec[3] = -q_vec[3];
    q_vec[4] = -q_vec[4]+M_PI; //check this for 3 wrist angs...
    while (q_vec[4]>DH_q_max5) q_vec[4]-=2.0*M_PI;
    q_vec[5] = q_vec[5];
    q_vec[6] = -q_vec[6]; //+M_PI;
    //while (q_vec[6]>DH_q_max7) q_vec[6]-=2.0*M_PI;

    //sled position is special case--what is displacement relative to part_y; negate this offset relative to part_y
    double d8_bin1= q_vec[7];
    //    d8 = y_part -Y_BASE_WRT_WORLD_AT_D8_HOME +fabs(dy_part_wrt_link0);
    double dsled = d8_bin1 - part_y + Y_BASE_WRT_WORLD_AT_D8_HOME;
    q_vec[7] =  part_y -Y_BASE_WRT_WORLD_AT_D8_HOME - dsled;
}

// set these member var values:
//current_hover_pose_
//approach_dropoff_jspace_pose_ = desired_approach_depart_pose_
//desired_grasp_dropoff_pose_
//
unsigned short int KukaBehaviorActionServer::compute_box_dropoff_key_poses(inventory_msgs::Part part) {
    //unsigned short int errorCode_ = robot_behavior_interface::RobotBehaviorResult::NO_ERROR;    
//  new...set these:
     //   Eigen::VectorXd  box_dropoff_hover_pose_, box_dropoff_cruise_pose_;
    //XXX COERCE THIS TO BOX Q1---FIX ME!!!
    part.location = Part::QUALITY_SENSOR_1;
    if (!hover_jspace_pose(part.location, current_hover_pose_)) {
        ROS_WARN("hover pose not recognized!");
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::WRONG_PARAMETER;
        return errorCode_;
    }
    
    box_dropoff_hover_pose_.resize(NDOF);
    box_dropoff_hover_pose_ = current_hover_pose_;
    box_dropoff_cruise_pose_.resize(NDOF);
    /*
             case Q1_HOVER_CODE:
            copy_array_to_qvec(Q1_HOVER_array,q_vec);
            return true;
      case Q2_HOVER_CODE:
            copy_array_to_qvec(Q2_HOVER_array,q_vec);
            return true;     */
    box_dropoff_cruise_pose_.resize(NDOF);
    
    if (Part::QUALITY_SENSOR_1==part.location) { 
        box_dropoff_cruise_pose_code_= Q1_CRUISE_CODE;
        box_dropoff_hover_pose_code_ = Q1_HOVER_CODE;
        copy_array_to_qvec(Q1_CRUISE_array,box_dropoff_cruise_pose_);
        copy_array_to_qvec(Q1_HOVER_array,box_dropoff_hover_pose_);
        
        ROS_INFO_STREAM("assigning cruise pose for Q1 to: "<<box_dropoff_cruise_pose_.transpose()<<endl);
        
    }
    else if (Part::QUALITY_SENSOR_2==part.location) {
        copy_array_to_qvec(Q2_CRUISE_array,box_dropoff_cruise_pose_);
        copy_array_to_qvec(Q2_HOVER_array,box_dropoff_hover_pose_);        
        box_dropoff_hover_pose_code_ = Q2_HOVER_CODE;

        ROS_INFO_STREAM("assigning cruise pose for Q2 to: "<<box_dropoff_cruise_pose_.transpose()<<endl);
        box_dropoff_cruise_pose_code_= Q2_CRUISE_CODE;

    }
    else {
        ROS_WARN("error--location should be a box location code to assign cruise pose");
        ROS_INFO("enter 1");
        int ans;
        cin>>ans;
    }

    
    //box_dropoff_cruise_pose_ = xxx
    
    
    
    //pickup_hover_pose_ = approx_jspace_pose; //remember this pose
    //current_hover_pose_ = pickup_hover_pose_;
    //ROS_INFO_STREAM("current_hover_pose_: "<<current_hover_pose_.transpose()<<endl);
    //computed_jspace_approach_.resize(8);
    geometry_msgs::PoseStamped part_pose_wrt_world = part.pose;
    Eigen::Affine3d affine_part_wrt_world = xformUtils_.transformPoseToEigenAffine3d(part_pose_wrt_world);
    Eigen::Vector3d O_part_wrt_world = affine_part_wrt_world.translation();
    ROS_INFO_STREAM("desired part origin w/rt world: "<<O_part_wrt_world.transpose()<<endl);

    //computed_jspace_approach_ contains pose estimate for  IKs
    //compute the IK for the desired pickup pose: pickup_jspace_pose_
    //first, get the equivalent desired affine of the vacuum gripper w/rt base_link;
    //need to provide the Part info and the rail displacement
    //Eigen::Affine3d RobotMoveActionServer::affine_vacuum_pickup_pose_wrt_base_link(Part part, double q_rail)
    affine_vacuum_pickup_pose_wrt_base_link_ = affine_vacuum_pickup_pose_wrt_base_link(part,
            current_hover_pose_[7]);
    
    //add a dropoff clearance tolerance to z value:
    Eigen::Vector3d O_dropoff_wrt_base_link = affine_vacuum_pickup_pose_wrt_base_link_.translation();
    adjust_box_place_limits(O_dropoff_wrt_base_link);
    affine_vacuum_pickup_pose_wrt_base_link_.translation() = O_dropoff_wrt_base_link;
            

    //provide desired gripper pose w/rt base_link, and choose soln closest to some reference jspace pose, e.g. hover pose
    //if (!get_pickup_IK(cart_grasp_pose_wrt_base_link,computed_jspace_approach_,&q_vec_soln);
    if (!compute_pickup_dropoff_IK(affine_vacuum_pickup_pose_wrt_base_link_, current_hover_pose_, desired_grasp_dropoff_pose_)) {
        ROS_WARN("could not compute IK soln for dropoff pose!");
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::UNREACHABLE;
        return errorCode_;
    }
    ROS_INFO_STREAM("desired_grasp_dropoff_pose_: " << desired_grasp_dropoff_pose_.transpose());

    //compute approach_pickup_jspace_pose_:  for approximate Cartesian descent and depart
    //compute_approach_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd computed_jspace_approach_,double approach_dist,Eigen::VectorXd &q_vec_soln);
    if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, desired_grasp_dropoff_pose_, approach_dist_,
            approach_dropoff_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for approach_dropoff_jspace_pose_!");
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::UNREACHABLE;
        return errorCode_;
    }
    //desired_approach_depart_pose_ = approach_dropoff_jspace_pose_;
    ROS_INFO_STREAM("approach_dropoff_jspace_pose_: " << approach_dropoff_jspace_pose_.transpose());
    
    return errorCode_;
        
}

//compute box_cam_grasp_inspection_pose_: pose to hold part in view of camera to get grasp transform
//also compute: box_dropoff_hover_pose_ (synonym) and box_dropoff_cruise_pose_
//ONLY look at the location code?  If have not seen box yet, can't trust world coords
//oops--ended up not using this;  it is still a part of alt_compute_box_dropoff_key_poses()
unsigned short int KukaBehaviorActionServer::box_cam_grasp_inspection_pose(inventory_msgs::Part part) {
    geometry_msgs::PoseStamped part_pose_wrt_world = part.pose;
    Eigen::Affine3d affine_part_wrt_world = xformUtils_.transformPoseToEigenAffine3d(part_pose_wrt_world);
    Eigen::Vector3d O_part_wrt_world = affine_part_wrt_world.translation();
    ROS_INFO_STREAM("desired part origin w/rt world: "<<O_part_wrt_world.transpose()<<endl);
    double destination_x_wrt_world = O_part_wrt_world[0];
    double destination_y_wrt_world = O_part_wrt_world[1];
    double J1_approach, J1_cruise;
    //ros::spinOnce(); //refresh joint states
    get_fresh_joint_states(); //update joint_state_vec_

    if (joint_state_vec_[0] >0) { 
        J1_approach= 2.8;
        J1_cruise = 1.57;
    }
    else { 
        J1_approach = -2.8; 
        J1_cruise = -1.57;
    };    
    //logic:
    //  *given desired dropoff  pose in world coords, and given J1, compute:
    // delta_d8 = -delta_x*tan(pi-|J1|)
    //*given desired gripper y w/rt world, compute sled displacement d8:
    //  d8 = y_gripper_wrt_world + delta_d8 + D8_..._HOME

    double delta_x = destination_x_wrt_world-X_BASE_WRT_WORLD;
    double delta_d8 = fabs(delta_x*tan(M_PI-fabs(J1_approach)));
    if (J1_approach<0) delta_d8 = -delta_d8; //correct the sign, depending on approach dir
    
    double d8 = destination_y_wrt_world+delta_d8 - Y_BASE_WRT_WORLD_AT_D8_HOME;
    
    if (d8>D8_MAX) d8=D8_MAX;
    if (d8<D8_MIN) d8=D8_MIN;
    ROS_INFO("using J1 = %f, delta_d8 = %f and d8 =  %f",J1_approach,delta_d8,d8);
    


    
    //use info to compute desired gripper pose w/rt base link
    copy_array_to_qvec(Q1_HOVER_array,box_dropoff_hover_pose_);
    copy_array_to_qvec(Q1_CRUISE_array,box_dropoff_cruise_pose_);
    box_dropoff_hover_pose_[0] = J1_approach;
    box_dropoff_hover_pose_[7] = d8;
    box_dropoff_cruise_pose_[0] = J1_cruise;
    box_dropoff_cruise_pose_[7] = d8;
    ROS_INFO_STREAM("box_dropoff_hover_pose_: " << box_dropoff_hover_pose_.transpose());
    box_cam_grasp_inspection_pose_.resize(8);
    box_cam_grasp_inspection_pose_=box_dropoff_hover_pose_;
        
    //check which box: compute an inspection pose that places robot's gripper in view of box cam
    if (part.location == Part::QUALITY_SENSOR_1)  {
        ROS_INFO("destination is box at station Q1");
        double d8_grasp_inspection = BOX_CAM_1_Y+delta_d8 - Y_BASE_WRT_WORLD_AT_D8_HOME;
        box_cam_grasp_inspection_pose_[7] = d8_grasp_inspection;

    }
    else if (part.location == Part::QUALITY_SENSOR_2) {
        ROS_INFO("destination is box at station Q2");    
        double d8_grasp_inspection = BOX_CAM_2_Y+delta_d8 - Y_BASE_WRT_WORLD_AT_D8_HOME;
        box_cam_grasp_inspection_pose_[7] = d8_grasp_inspection;
    }
    else {
        ROS_WARN("alt_compute_box_dropoff_key_poses()");
        ROS_WARN("destination code is not Q1 nor Q2!");
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::WRONG_PARAMETER;
        return errorCode_;
    }    
    ROS_INFO_STREAM("box_cam_grasp_inspection_pose_: " << box_cam_grasp_inspection_pose_.transpose());
    errorCode_ = robot_behavior_interface::RobotBehaviorResult::NO_ERROR;
    return errorCode_;
    
}


//compute these key poses:
//box_dropoff_cruise_pose_, box_dropoff_hover_pose_=box_cam_grasp_inspection_pose_, desired_grasp_dropoff_pose_, approach_dropoff_jspace_pose_, 
//also, pickup_deeper_jspace_pose_, useful for pick-part-from-box
unsigned short int KukaBehaviorActionServer::alt_compute_box_dropoff_key_poses(inventory_msgs::Part part) {
    //unsigned short int errorCode_ = robot_behavior_interface::RobotBehaviorResult::NO_ERROR;    
//  new...set these:
    
    geometry_msgs::PoseStamped part_pose_wrt_world = part.pose;
    Eigen::Affine3d affine_part_wrt_world = xformUtils_.transformPoseToEigenAffine3d(part_pose_wrt_world);
    Eigen::Vector3d O_part_wrt_world = affine_part_wrt_world.translation();
    ROS_INFO_STREAM("alt_compute_box_dropoff_key_poses: part is:"<<endl<<part<<endl);
    ROS_INFO_STREAM("desired part origin w/rt world: "<<O_part_wrt_world.transpose()<<endl);
    double destination_x_wrt_world = O_part_wrt_world[0];
    double destination_y_wrt_world = O_part_wrt_world[1];
    double J1_approach, J1_cruise;
    //ros::spinOnce(); //refresh joint states
    get_fresh_joint_states(); //update joint_state_vec_

    if (joint_state_vec_[0] >0) { 
        J1_approach= 2.8;
        J1_cruise = 1.57;
    }
    else { 
        J1_approach = -2.8; 
        J1_cruise = -1.57;
    };    
    //logic:
    //  *given desired dropoff  pose in world coords, and given J1, compute:
    // delta_d8 = -delta_x*tan(pi-|J1|)
    //*given desired gripper y w/rt world, compute sled displacement d8:
    //  d8 = y_gripper_wrt_world + delta_d8 + D8_..._HOME

    double delta_x = destination_x_wrt_world-X_BASE_WRT_WORLD;
    double delta_d8 = fabs(delta_x*tan(M_PI-fabs(J1_approach)));
    if (J1_approach<0) delta_d8 = -delta_d8; //correct the sign, depending on approach dir
    
    double d8 = destination_y_wrt_world+delta_d8 - Y_BASE_WRT_WORLD_AT_D8_HOME;
    
    if (d8>D8_MAX) d8=D8_MAX;
    if (d8<D8_MIN) d8=D8_MIN;
    ROS_INFO("using J1 = %f, delta_d8 = %f and d8 =  %f",J1_approach,delta_d8,d8);
    


    
    //use info to compute desired gripper pose w/rt base link
    copy_array_to_qvec(Q1_HOVER_array,box_dropoff_hover_pose_);
    copy_array_to_qvec(Q1_CRUISE_array,box_dropoff_cruise_pose_);
    box_dropoff_hover_pose_[0] = J1_approach;
    box_dropoff_hover_pose_[7] = d8;
    box_dropoff_cruise_pose_[0] = J1_cruise;
    box_dropoff_cruise_pose_[7] = d8;
    ROS_INFO_STREAM("box_dropoff_hover_pose_: " << box_dropoff_hover_pose_.transpose());
    box_cam_grasp_inspection_pose_.resize(8);
    box_cam_grasp_inspection_pose_=box_dropoff_hover_pose_;
        
    //check which box: compute an inspection pose that places robot's gripper in view of box cam
    if (part.location == Part::QUALITY_SENSOR_1)  {
        ROS_INFO("destination is box at station Q1");
        double d8_grasp_inspection = BOX_CAM_1_Y+delta_d8 - Y_BASE_WRT_WORLD_AT_D8_HOME;
        box_cam_grasp_inspection_pose_[7] = d8_grasp_inspection;

    }
    else if (part.location == Part::QUALITY_SENSOR_2) {
        ROS_INFO("destination is box at station Q2");    
        double d8_grasp_inspection = BOX_CAM_2_Y+delta_d8 - Y_BASE_WRT_WORLD_AT_D8_HOME;
        box_cam_grasp_inspection_pose_[7] = d8_grasp_inspection;
    }
    else {
        ROS_WARN("alt_compute_box_dropoff_key_poses()");
        ROS_WARN("destination code is not Q1 nor Q2!");
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::WRONG_PARAMETER;
        return errorCode_;
    }    
    ROS_INFO_STREAM("box_cam_grasp_inspection_pose_: " << box_cam_grasp_inspection_pose_.transpose());

    //computed_jspace_approach_ contains pose estimate for  IKs
    //compute the IK for the desired pickup pose: pickup_jspace_pose_
    //first, get the equivalent desired affine of the vacuum gripper w/rt base_link;
    //need to provide the Part info and the rail displacement
    //Eigen::Affine3d RobotMoveActionServer::affine_vacuum_pickup_pose_wrt_base_link(Part part, double q_rail)
    affine_vacuum_pickup_pose_wrt_base_link_ = affine_vacuum_pickup_pose_wrt_base_link(part,d8);
    
    //add a dropoff clearance tolerance to z value:
    Eigen::Vector3d O_dropoff_wrt_base_link = affine_vacuum_pickup_pose_wrt_base_link_.translation();
    adjust_box_place_limits(O_dropoff_wrt_base_link);
    affine_vacuum_pickup_pose_wrt_base_link_.translation() = O_dropoff_wrt_base_link;
            

    //provide desired gripper pose w/rt base_link, and choose soln closest to some reference jspace pose, e.g. hover pose
    //if (!get_pickup_IK(cart_grasp_pose_wrt_base_link,computed_jspace_approach_,&q_vec_soln);
    if (!compute_pickup_dropoff_IK(affine_vacuum_pickup_pose_wrt_base_link_, box_dropoff_hover_pose_, desired_grasp_dropoff_pose_)) {
        ROS_WARN("could not compute IK soln for dropoff pose!");
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::UNREACHABLE;
        return errorCode_;
    }
    ROS_INFO_STREAM("desired_grasp_dropoff_pose_: " << desired_grasp_dropoff_pose_.transpose());

    //compute approach_pickup_jspace_pose_:  for approximate Cartesian descent and depart
    //compute_approach_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd computed_jspace_approach_,double approach_dist,Eigen::VectorXd &q_vec_soln);
    if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, desired_grasp_dropoff_pose_, approach_dist_,
            approach_dropoff_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for approach_dropoff_jspace_pose_!");
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::UNREACHABLE;
        return errorCode_;
    }
    //use deep grasp pose for pick-part from box:
    if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, desired_grasp_dropoff_pose_, deep_grasp_dist_,
            pickup_deeper_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for deep pickup approach pose");
        ROS_WARN("re-using grasp pose; not returning an error");
        pickup_deeper_jspace_pose_ = pickup_jspace_pose_;
        //errorCode_ = robot_behavior_interface::RobotBehaviorResult::UNREACHABLE;
        //return errorCode_;
    }    
    //desired_approach_depart_pose_ = approach_dropoff_jspace_pose_;
    ROS_INFO_STREAM("approach_dropoff_jspace_pose_: " << approach_dropoff_jspace_pose_.transpose());
    ROS_INFO("done computing key poses  for box dropoff");
    errorCode_ = robot_behavior_interface::RobotBehaviorResult::NO_ERROR;
    return errorCode_;
        
}

//Identical to  above, except provide computed pickup_pose (from bin-pick) instead of current arm pose
// 
//compute these values: box_dropoff_cruise_pose_, box_dropoff_hover_pose_, box_cam_grasp_inspection_pose_
// approach_dropoff_jspace_pose_, desired_grasp_dropoff_pose_,
//  pickup_jspace_pose is used to establish righty/lefty
unsigned short int KukaBehaviorActionServer::plan_box_dropoff_key_poses(inventory_msgs::Part part, Eigen::VectorXd pickup_jspace_pose) {
    //unsigned short int errorCode_ = robot_behavior_interface::RobotBehaviorResult::NO_ERROR;    
//  new...set these:
    
    geometry_msgs::PoseStamped part_pose_wrt_world = part.pose;
    Eigen::Affine3d affine_part_wrt_world = xformUtils_.transformPoseToEigenAffine3d(part_pose_wrt_world);
    Eigen::Vector3d O_part_wrt_world = affine_part_wrt_world.translation();
    ROS_INFO_STREAM("desired part origin w/rt world: "<<O_part_wrt_world.transpose()<<endl);
    double destination_x_wrt_world = O_part_wrt_world[0];
    double destination_y_wrt_world = O_part_wrt_world[1];
    double J1_approach, J1_cruise;

    if (pickup_jspace_pose[0] >0) { 
        J1_approach= 2.8;
        J1_cruise = 1.57;
    }
    else { 
        J1_approach = -2.8; 
        J1_cruise = -1.57;
    };    
    //logic:
    //  *given desired dropoff  pose in world coords, and given J1, compute:
    // delta_d8 = -delta_x*tan(pi-|J1|)
    //*given desired gripper y w/rt world, compute sled displacement d8:
    //  d8 = y_gripper_wrt_world + delta_d8 + D8_..._HOME

    double delta_x = destination_x_wrt_world-X_BASE_WRT_WORLD;
    double delta_d8 = fabs(delta_x*tan(M_PI-fabs(J1_approach)));
    if (J1_approach<0) delta_d8 = -delta_d8; //correct the sign, depending on approach dir
    
    double d8 = destination_y_wrt_world+delta_d8 - Y_BASE_WRT_WORLD_AT_D8_HOME;
    
    if (d8>D8_MAX) d8=D8_MAX;
    if (d8<D8_MIN) d8=D8_MIN;
    ROS_INFO("using J1 = %f, delta_d8 = %f and d8 =  %f",J1_approach,delta_d8,d8);
    
    //use info to compute desired gripper pose w/rt base link
    copy_array_to_qvec(Q1_HOVER_array,box_dropoff_hover_pose_);
    copy_array_to_qvec(Q1_CRUISE_array,box_dropoff_cruise_pose_);
    box_dropoff_hover_pose_[0] = J1_approach;
    box_dropoff_hover_pose_[7] = d8;
    box_dropoff_cruise_pose_[0] = J1_cruise;
    box_dropoff_cruise_pose_[7] = d8;
    ROS_INFO_STREAM("box_dropoff_hover_pose_: " << box_dropoff_hover_pose_.transpose());
    box_cam_grasp_inspection_pose_.resize(8);
    box_cam_grasp_inspection_pose_=box_dropoff_hover_pose_;
        
    //check which box: compute an inspection pose that places robot's gripper in view of box cam
    if (part.location == Part::QUALITY_SENSOR_1)  {
        ROS_INFO("destination is box at station Q1");
        double d8_grasp_inspection = BOX_CAM_1_Y+delta_d8 - Y_BASE_WRT_WORLD_AT_D8_HOME;
        box_cam_grasp_inspection_pose_[7] = d8_grasp_inspection;

    }
    else if (part.location == Part::QUALITY_SENSOR_2) {
        ROS_INFO("destination is box at station Q2");    
        double d8_grasp_inspection = BOX_CAM_2_Y+delta_d8 - Y_BASE_WRT_WORLD_AT_D8_HOME;
        box_cam_grasp_inspection_pose_[7] = d8_grasp_inspection;
    }
    else {
        ROS_WARN("alt_compute_box_dropoff_key_poses()");
        ROS_WARN("destination code is not Q1 nor Q2!");
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::WRONG_PARAMETER;
        return errorCode_;
    }    
    ROS_INFO_STREAM("box_cam_grasp_inspection_pose_: " << box_cam_grasp_inspection_pose_.transpose());

    //computed_jspace_approach_ contains pose estimate for  IKs
    //compute the IK for the desired pickup pose: pickup_jspace_pose_
    //first, get the equivalent desired affine of the vacuum gripper w/rt base_link;
    //need to provide the Part info and the rail displacement
    //Eigen::Affine3d RobotMoveActionServer::affine_vacuum_pickup_pose_wrt_base_link(Part part, double q_rail)
    affine_vacuum_pickup_pose_wrt_base_link_ = affine_vacuum_pickup_pose_wrt_base_link(part,d8);
    
    //add a dropoff clearance tolerance to z value:
    Eigen::Vector3d O_dropoff_wrt_base_link = affine_vacuum_pickup_pose_wrt_base_link_.translation();
    adjust_box_place_limits(O_dropoff_wrt_base_link);
    affine_vacuum_pickup_pose_wrt_base_link_.translation() = O_dropoff_wrt_base_link;
            

    //provide desired gripper pose w/rt base_link, and choose soln closest to some reference jspace pose, e.g. hover pose
    //if (!get_pickup_IK(cart_grasp_pose_wrt_base_link,computed_jspace_approach_,&q_vec_soln);
    if (!compute_pickup_dropoff_IK(affine_vacuum_pickup_pose_wrt_base_link_, box_dropoff_hover_pose_, desired_grasp_dropoff_pose_)) {
        ROS_WARN("could not compute IK soln for dropoff pose!");
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::UNREACHABLE;
        return errorCode_;
    }
    ROS_INFO_STREAM("desired_grasp_dropoff_pose_: " << desired_grasp_dropoff_pose_.transpose());

    //compute approach_pickup_jspace_pose_:  for approximate Cartesian descent and depart
    //compute_approach_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd computed_jspace_approach_,double approach_dist,Eigen::VectorXd &q_vec_soln);
    if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, desired_grasp_dropoff_pose_, approach_dist_,
            approach_dropoff_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for approach_dropoff_jspace_pose_!");
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::UNREACHABLE;
        return errorCode_;
    }
    //use deep grasp pose for pick-part from box:
    if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, desired_grasp_dropoff_pose_, deep_grasp_dist_,
            pickup_deeper_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for deep pickup approach pose");
        ROS_WARN("re-using grasp pose; not returning an error");
        pickup_deeper_jspace_pose_ = pickup_jspace_pose_;
        //errorCode_ = robot_behavior_interface::RobotBehaviorResult::UNREACHABLE;
        //return errorCode_;
    }    
    //desired_approach_depart_pose_ = approach_dropoff_jspace_pose_;
    ROS_INFO_STREAM("approach_dropoff_jspace_pose_: " << approach_dropoff_jspace_pose_.transpose());
    ROS_INFO("done computing key poses  for box dropoff");
    errorCode_ = robot_behavior_interface::RobotBehaviorResult::NO_ERROR;
    return errorCode_;
        
}

bool KukaBehaviorActionServer::bin_center_y_coord(int8_t location, double &bin_y_val) {
    //bin y locations are: -0.49, 0.32, 1.13, 1.94, 2.75;
    switch (location) {
        case Part::BIN1:
            bin_y_val = -0.49; //extract rail position for bin1 key pose
            break;
        case Part::BIN2:
            bin_y_val = 0.32; //extract rail position for bin1 key pose
            break;
        case Part::BIN3:
            bin_y_val = 1.13; //extract rail position for bin1 key pose
            break;
        case Part::BIN4:
            bin_y_val = 1.94; //extract rail position for bin1 key pose
            break;
        case Part::BIN5:
            bin_y_val = 2.75; //extract rail position for bin1 key pose
            break;        
        default:
            ROS_WARN("unrecognized location code");
            return false;
    }
    return true; // if here, got valid bin code and filled in q_rail
}

//for picking parts from bins, find a good value hover pose, given x,y values of
// part w/rt world; use the rail to help with pose
bool KukaBehaviorActionServer::compute_bin_hover_from_xy(double x_part,double y_part, Eigen::VectorXd &qvec) {
    qvec.resize(8);
    double d8,J1_ang;
    //some magic numbers: bin locations--> x_frontrow = -0.651; x_midrow = -0.776; x_backrow=-0.89
    /* already done by bin_xy_is_reachable() 
    if (x_part< MIN_BIN_X_VAL) {
        ROS_WARN("part is too far away to reach");
        x_part = MIN_BIN_X_VAL;
        ROS_INFO("recommend trying x_part = %f", x_part);
        return false;    
    }
    if (x_part> MAX_BIN_X_VAL) {
        ROS_WARN("something wrong--part is too close to front of bin");
        x_part = MAX_BIN_X_VAL;
        ROS_INFO("recommend trying x_part = %f", x_part);        
        return false;
    }
    */
    if (x_part< BIN_FAR_THRESHOLD) { //FAR--backrow case:
        //this pose has arm nearly fully outstretched and J1 rotated to face part, nearly  head-on
        //adjust slightly, so approach is partly angled, reflected from non-flipped approach
        qvec<<0.21, -1.65, 0.0, 0.1, 0, -1.2, 0, -0.527;
        d8 = y_part -Y_BASE_WRT_WORLD_AT_D8_HOME + 0.14; //head-on approach; line up sled with part y
        qvec[7]=d8;
        return true;
    }
    //else, x is not backrow; compute values for J1_ang and d8, freezing other joints:
    double radius = 0.77; //this is radius from base to gripper at proposed arm pose
    //slightly bent elbow, suitable for access to hover over bin;
    //radius above corresponds to this pose of shoulder and elbow angles
    qvec<<0.7, -1.35, 0, 0.6, -0.113, -0.807, 2.8085, -0.355; // left front part approach
    //center front; 0.7, -1.35, 0, 0.8, -0.113, -0.807, 2.8085, -0.18
    //consider a 2-DOF robot with one revolute and one prismatic joint;
    //J1 is the revolute joint, and the track is the prismatic joint
    //compute J1 and d8 to place the gripper over a part at give  (x,y), assuming
    //elbow and shoulder angles as chosen above  (--> radius)
    //formula: fabs(dx)/RADIUS = sin(pi/2-theta_J1) 
    // theta_J1 = pi/2 - asin(fabs(dx)/RADIUS) 
    double dx = fabs(x_part-X_BASE_WRT_WORLD); //dist from rail to part
    J1_ang = M_PI/2.0 - asin(dx/radius);
    //AND: dy_part_wrt_link0 = RADIUS*cos(1.57-theta_J1)
    //d8 = y_part_wrt_world - fabs(dy_part_wrt_link0)
    double dy_part_wrt_link0 = radius*cos(M_PI/2.0 -J1_ang);
    d8 = y_part -Y_BASE_WRT_WORLD_AT_D8_HOME +fabs(dy_part_wrt_link0);
    ROS_INFO("y_part = %f, dy_part_wrt_link0 = %f, d8 = %f",y_part,dy_part_wrt_link0, d8);
            
    qvec[0]=J1_ang;
    qvec[7] = d8;
    
    return true;
}
 
bool KukaBehaviorActionServer::bin_xy_is_reachable(int8_t bin,double &part_x, double &part_y) {
    double bin_center_y;
    bool rtn_val = true;
    if (!bin_center_y_coord(bin, bin_center_y)) {
        ROS_WARN("error--bin code not recognized");
        return false;
    }
    //eval if proposed grasp y-coord would clear the bin or not:
    //require 0.190 > y_part-y_bin >  -0.153
    double dy = part_y - bin_center_y;
    if (dy< MIN_BIN_GRASP_DY) {
        part_y= MIN_BIN_GRASP_DY+bin_center_y;
        ROS_WARN("part is too close to bin left side; proposing new y_val = %f",part_y);
        rtn_val= false;
    }
    
    if (dy> MAX_BIN_GRASP_DY) {
        part_y= MAX_BIN_GRASP_DY+bin_center_y;
        ROS_WARN("part is too close to bin right side; proposing new y_val = %f",part_y);
        rtn_val =  false;
    }    
    
    if (part_x< MIN_BIN_X_VAL) {
        ROS_WARN("part is too far away to reach");
        part_x = MIN_BIN_X_VAL;
        ROS_INFO("recommend trying x_part = %f", part_x);
        rtn_val = false;    
    }
    if (part_x> MAX_BIN_X_VAL) {
        ROS_WARN("something wrong--part is too close to front of bin");
        part_x = MAX_BIN_X_VAL;
        ROS_INFO("recommend trying x_part = %f", part_x);        
        rtn_val = false;
    }    
    return rtn_val;
    
}

//for each of the 10 key poses, extract the rail position

bool KukaBehaviorActionServer::rail_prepose(int8_t location, double &q_rail) { 
    switch (location) {
        case Part::BIN1:
            q_rail = BIN1_HOVER_FAR_array[7]; //extract rail position for bin1 key pose
            break;
        case Part::BIN2:
            q_rail = BIN2_HOVER_NEAR_array[7]; //extract rail position for bin1 key pose
            break;
        case Part::BIN3:
            q_rail = BIN3_HOVER_FAR_array[7]; //extract rail position for bin1 key pose
            break;
        case Part::BIN4:
            q_rail = BIN4_HOVER_FAR_array[7]; //extract rail position for bin1 key pose
            break;
        case Part::BIN5:
            q_rail = BIN5_HOVER_FAR_array[7]; //extract rail position for bin1 key pose
            break;
            /*
        case Part::BIN6:
            q_rail = q_bin6_hover_pose_[1]; //extract rail position for bin1 key pose
            break;
        case Part::BIN7:
            q_rail = q_bin7_hover_pose_[1]; //extract rail position for bin1 key pose
            break;
        case Part::BIN8:
            q_rail = q_bin8_hover_pose_[1]; //extract rail position for bin1 key pose
            break;
             * */
        case Part::QUALITY_SENSOR_1:
            q_rail = Q1_HOVER_array[7]; //extract rail position for key pose
            break;     
        //case Part::QUALITY_SENSOR_2:
        //    q_rail = q_Q2_fwd_pose_[1]; //extract rail position for key pose
        //    break;               
        default:
            ROS_WARN("unrecognized location code");
            return false;
    }
    return true; // if here, got valid bin code and filled in q_rail
}



void KukaBehaviorActionServer::copy_array_to_qvec(const double q_array[],Eigen::VectorXd &qvec) {
  qvec.resize(NDOF);
  for (int i=0;i<NDOF;i++) {
    qvec[i] = q_array[i];
  }
}

bool KukaBehaviorActionServer::hover_jspace_pose_from_pose_code(int pose_code, Eigen::VectorXd &q_vec) {
  switch (pose_code) {
        case Q1_HOVER_CODE:
            copy_array_to_qvec(Q1_HOVER_array,q_vec);
            return true;
      case Q2_HOVER_CODE:
            copy_array_to_qvec(Q2_HOVER_array,q_vec);
            return true;       
       default:
            ROS_WARN("location code not recognized");
            return false;   
  }
}

bool KukaBehaviorActionServer::hover_jspace_pose(int8_t bin, Eigen::VectorXd &q_vec) {
    unsigned short int box_placement_location_code=0; //obsolete--unused
    return hover_jspace_pose_w_code(bin, box_placement_location_code,q_vec);
}


bool KukaBehaviorActionServer::hover_jspace_pose_w_code(int8_t bin, unsigned short int box_placement_location_code, Eigen::VectorXd &qvec) {
    switch (bin) {
        case Part::BIN1:
            copy_array_to_qvec(BIN1_HOVER_FAR_array,qvec);
            return true; //valid code
            break;
        case Part::BIN2:
            copy_array_to_qvec(BIN2_HOVER_NEAR_array,qvec);
            return true; //valid code
            break;
        case Part::BIN3:
            copy_array_to_qvec(BIN3_HOVER_FAR_array,qvec);
            return true; //valid code
            break;
        case Part::BIN4:
            copy_array_to_qvec(BIN4_HOVER_FAR_array,qvec);
            return true; //valid code
            break;
        case Part::BIN5:
            copy_array_to_qvec(BIN5_HOVER_FAR_array,qvec);
            return true; //valid code
            break;
            /*
        case Part::BIN6:
            qvec = q_bin6_hover_pose_;
            return true; //valid code
            break;
        case Part::BIN7:
            qvec = q_bin7_hover_pose_;
            return true; //valid code
            break;
        case Part::BIN8:
            qvec = q_bin8_hover_pose_;
            return true; //valid code
            break;
             * */
        case Part::QUALITY_SENSOR_1: //4 different drop-off hover poses, depending on placement location in box
            ROS_INFO("destination location QUALITY_SENSOR_1");
            copy_array_to_qvec(Q1_HOVER_array,qvec);

            /*
            if (box_placement_location_code==robot_behavior_interface::RobotMoveGoal::PART_FAR_LEFT) {
                unsigned short int hover_code = robot_behavior_interface::RobotMoveGoal::Q1_HOVER_POSE_LEFT_FAR;
                get_pose_from_code(hover_code, qvec);                 
                //qvec = q_Q1_righty_hover_;
                return true;
            }
            if (box_placement_location_code==robot_behavior_interface::RobotMoveGoal::PART_FAR_RIGHT) {
                ROS_INFO("place part far right");
                get_pose_from_code(robot_behavior_interface::RobotMoveGoal::Q1_HOVER_POSE_RIGHT_FAR, qvec);               
                //qvec = q_Q1_righty_hover_;
                return true;
            }
            if (box_placement_location_code==robot_behavior_interface::RobotMoveGoal::PART_NEAR_LEFT) {
                get_pose_from_code(robot_behavior_interface::RobotMoveGoal::Q1_HOVER_POSE_LEFT_NEAR, qvec);
                //qvec = q_Q1_righty_hover_; //Q1_RIGHTY_HOVER_FLIP
                return true;
            }
            if (box_placement_location_code==robot_behavior_interface::RobotMoveGoal::PART_FAR_LEFT ) {
                get_pose_from_code(robot_behavior_interface::RobotMoveGoal::Q1_HOVER_POSE_LEFT_FAR, qvec);
                //qvec = q_Q1_righty_hover_;//Q1_RIGHTY_HOVER
                return true;
            }*/

            //if here, failed            
            //ROS_WARN("dropoff location code not recognized!");
            return true;
            break;
            /*
        case Part::QUALITY_SENSOR_2:
             copy_array_to_qvec(Q2_HOVER_array,qvec);
            return true; //valid code
            break;           */ 
        default:
            ROS_WARN("location code not recognized");
            return false;
    }
}


/*
    bool KukaBehaviorActionServer::set_q_manip_nom_from_destination_part(Part part) {
        unsigned short int box_placement_location_code = part.box_placement_location_code;
     switch (part.location) {       
        case Part::QUALITY_SENSOR_1: //4 different drop-off hover poses, depending on placement location in box
            ROS_INFO("destination location QUALITY_SENSOR_1");
            if (box_placement_location_code==robot_behavior_interface::RobotMoveGoal::PART_NEAR_RIGHT) {
                q_manip_nom_ = q_Q1_dropoff_near_right_;                
                return true;
            }
            if (box_placement_location_code==robot_behavior_interface::RobotMoveGoal::PART_FAR_RIGHT) {
                q_manip_nom_ = q_Q1_dropoff_far_right_;                //qvec = q_Q1_righty_hover_;
                return true;
            }
            if (box_placement_location_code==robot_behavior_interface::RobotMoveGoal::PART_NEAR_LEFT) {
                q_manip_nom_ = q_Q1_dropoff_near_left_;
                return true;
            }
            if (box_placement_location_code==robot_behavior_interface::RobotMoveGoal::PART_FAR_LEFT ) {
                q_manip_nom_ = q_Q1_dropoff_far_left_;
                return true;
            }        

        default:
            ROS_WARN("Q1 nom dropoff code not recognized");
            return false;
    }
}

bool KukaBehaviorActionServer::cruise_jspace_pose(int8_t bin,  Eigen::VectorXd &q_vec) {
    unsigned short int dropoff_code = robot_behavior_interface::RobotMoveGoal::Q1_NOM_DROPOFF_POSE_UNKNOWN;
    return cruise_jspace_pose_w_code(bin,dropoff_code,q_vec);
}; // { }; //default, no box location code

bool KukaBehaviorActionServer::cruise_jspace_pose_w_code(int8_t location,unsigned short int box_placement_location_code, Eigen::VectorXd &q_vec) {
    switch (location) {
        case Part::BIN1:
            q_vec = q_bin1_cruise_pose_; //shape is good, but change sled position
            break;
        case Part::BIN2:    
            q_vec = q_bin1_cruise_pose_; //shape is good, but change sled position
            q_vec[1] = q_bin2_hover_pose_[1]; //set rail position same as hover pose
            break;            
        case Part::BIN3:    
            q_vec = q_bin1_cruise_pose_; //shape is good, but change sled position
            q_vec[1] = q_bin3_hover_pose_[1]; //set rail position same as hover pose
            break;             
        case Part::BIN4:    
            q_vec = q_bin1_cruise_pose_; //shape is good, but change sled position
            q_vec[1] = q_bin4_hover_pose_[1]; //set rail position same as hover pose
            break;  
        case Part::BIN5:    
            q_vec = q_bin1_cruise_pose_; //shape is good, but change sled position
            q_vec[1] = q_bin5_hover_pose_[1]; //set rail position same as hover pose
            break;  
        case Part::BIN6:    
            q_vec = q_bin1_cruise_pose_; //shape is good, but change sled position
            q_vec[1] = q_bin6_hover_pose_[1]; //set rail position same as hover pose
            break;  
        case Part::BIN7:    
            q_vec = q_bin1_cruise_pose_; //shape is good, but change sled position
            q_vec[1] = q_bin7_hover_pose_[1]; //set rail position same as hover pose
            break;  
        case Part::BIN8:    
            q_vec = q_bin1_cruise_pose_; //shape is good, but change sled position
            q_vec[1] = q_bin8_hover_pose_[1]; //set rail position same as hover pose
            break;              
        case Part::QUALITY_SENSOR_1:
            //FIX ME!
            if (box_placement_location_code==robot_behavior_interface::RobotMoveGoal::PART_NEAR_RIGHT) {
                get_pose_from_code(robot_behavior_interface::RobotMoveGoal::Q1_RIGHT_CRUISE_POSE, q_vec);                 
                //qvec = q_Q1_righty_hover_;
                return true;
            }
            if (box_placement_location_code==robot_behavior_interface::RobotMoveGoal::PART_FAR_RIGHT) {
                //Q1_LEFTY_HOVER_FLIP
                get_pose_from_code(robot_behavior_interface::RobotMoveGoal::Q1_RIGHT_CRUISE_POSE, q_vec);               
                //qvec = q_Q1_righty_hover_;
                return true;
            }
            if (box_placement_location_code==robot_behavior_interface::RobotMoveGoal::PART_NEAR_LEFT) {
                get_pose_from_code(robot_behavior_interface::RobotMoveGoal::Q1_LEFT_CRUISE_POSE, q_vec);
                //qvec = q_Q1_righty_hover_; //Q1_RIGHTY_HOVER_FLIP
                return true;
            }
            if (box_placement_location_code==robot_behavior_interface::RobotMoveGoal::PART_FAR_LEFT ) {
                get_pose_from_code(robot_behavior_interface::RobotMoveGoal::Q1_LEFT_CRUISE_POSE, q_vec);
                //qvec = q_Q1_righty_hover_;//Q1_RIGHTY_HOVER
                return true;
            }
            //if here, failed            
            ROS_WARN("dropoff location code not recognized!");
            return false;
            break;            
      
        case Part::QUALITY_SENSOR_2:
            q_vec = q_box_Q2_cruise_pose_;
            break; 

        default:
            ROS_WARN("unknown destination code");
            return false;
    }
    return true;
}
*/

//3/29/17 new function, generalizes on pickup offset;
//w/ gasket, cannot pick up part at part origin, due to hole in center
//define a desired transform between gripper frame and part frame
// this should include part thickness as part of any necessary displacement from part origin
// extend this to return grasp transforms for inverted parts
//return grasp_transform, which is part pose w/rt gripper for vacuum grasp
bool KukaBehaviorActionServer::get_grasp_transform(Part part, Eigen::Affine3d &grasp_transform) {
//Eigen::Affine3d grasp_transform;
    bool part_is_up = eval_up_down(part.pose);
    if (!part_is_up) ROS_WARN("part is inverted");
    Eigen::Matrix3d R, R_inverted;
    R = Eigen::MatrixXd::Identity(3, 3);
    R_inverted = R; //rot pi about x--> x is 1,0,0; y = 0, -1, 0; z = 0,0,-1
    Eigen::Vector3d y_inv,z_inv;
    y_inv<<0,-1,0;
    z_inv<<0,0,-1;
    R_inverted.col(1) = y_inv;
    R_inverted.col(2) = z_inv;
    Eigen::Vector3d O_part_wrt_gripper;
    O_part_wrt_gripper << 0, 0, 0; //= Eigen::MatrixXd::Zero(3, 1);
    grasp_transform.linear() = R;
    //default: transform is identity, zero offset--> part frame = gripper frame
    grasp_transform.translation() = O_part_wrt_gripper;

    string part_name(part.name); //a C++ string
    if (part_name.compare("gear_part") == 0) {
      if(part_is_up) {
        O_part_wrt_gripper[2] = -(GEAR_PART_THICKNESS);
        O_part_wrt_gripper[1] = GEAR_PART_GRASP_Y_OFFSET; // offset to avoid touching dowell
        grasp_transform.translation() = O_part_wrt_gripper; 
        ROS_INFO_STREAM("gear part: O_part_wrt_gripper = "<<O_part_wrt_gripper.transpose()<<endl);
        return true;
       }
      else {
        ROS_WARN("inverted grasp not defined for this part");
        return false;
      }
    }
    //piston_rod_part
    if (part_name.compare("piston_rod_part") == 0) {
       if(part_is_up) {
        O_part_wrt_gripper[2] = -(PISTON_ROD_PART_THICKNESS + 0.003);
        grasp_transform.translation() = O_part_wrt_gripper;
        return true;
       }
      else {
        ROS_WARN("inverted grasp not defined for this part");
        return false;
      }
    }
    //disk_part
    if (part_name.compare("disk_part") == 0) {
       if(part_is_up) {
        O_part_wrt_gripper[2] = -(DISK_PART_THICKNESS + DISK_PART_GRASP_Z_OFFSET);
        grasp_transform.translation() = O_part_wrt_gripper;
        return true;
       }
      else {
        ROS_WARN("inverted grasp not defined for this part");
        return false;
      }
    }
    //gasket_part
    if (part_name.compare("gasket_part") == 0) {
       if(part_is_up) {
        O_part_wrt_gripper[2] = -(GASKET_PART_THICKNESS)+GASKET_PART_GRASP_Z_OFFSET; // manual tweak for grasp from conveyor
        //for gasket, CANNOT grab at center!! there is a hole there
        O_part_wrt_gripper[0] = GASKET_PART_GRASP_X_OFFSET; // TUNE ME; if negative, then hit
        grasp_transform.translation() = O_part_wrt_gripper;
        return true;
       }
      else {
        ROS_WARN("inverted grasp not defined for this part");
        return false;
      }
    }
    if (part_name.compare("pulley_part") == 0) {
       if(part_is_up) {
        O_part_wrt_gripper[2] = -(PULLEY_PART_THICKNESS + PULLEY_PART_GRASP_Z_OFFSET);
        grasp_transform.translation() = O_part_wrt_gripper;
        return true;
       }
      else {
        ROS_WARN("using inverted pulley-part grasp transform");
        O_part_wrt_gripper[2] = -PULLEY_PART_GRASP_Z_OFFSET-PULLEY_PART_GRASP_Z_OFFSET; //-(PULLEY_PART_THICKNESS + PULLEY_PART_GRASP_Z_OFFSET);
        grasp_transform.translation() = O_part_wrt_gripper;
        grasp_transform.linear() =  R_inverted;
        return true;
      }
    }
    ROS_WARN("get_grasp_transform: part name not recognized: %s", part.name.c_str());
    return false; // don't recognize part, so just return zero

}


//for each part, there is a vertical offset from the part frame to the gripper frame on top surface
//return this value; only needs part.name
//******** generalize this to get T_grasp = T_part_frame/gripper_frame



double KukaBehaviorActionServer::get_pickup_offset(Part part) {
    double offset;
    string part_name(part.name); //a C++ string
    if (part_name.compare("gear_part") == 0) {
        offset = GEAR_PART_THICKNESS +
                 0.007; //0.007 seems perfect, within 1mm, for bin; but had to add addl 4mm for pickup from tray
        return offset;
    }
    //piston_rod_part
    if (part_name.compare("piston_rod_part") == 0) {
        offset = PISTON_ROD_PART_THICKNESS + 0.005; //0.005 correction looks very good for pickup from bin
        return offset;
    }
    //disk_part
    if (part_name.compare("disk_part") == 0) {
        offset = DISK_PART_THICKNESS + 0.005; //0.005 correction looks very good for pickup from bin
        return offset;
    }
    //gasket_part
    if (part_name.compare("gasket_part") == 0) {
        offset = GASKET_PART_THICKNESS + 0.005; //try adjusting for conveyor pickup
        return offset;
    }

    ROS_WARN("part name not recognized");
    return 0.0; // don't recognize part, so just return zero

}

/*
double KukaBehaviorActionServer::get_surface_height(Part part) {
    switch (part.location) {
        case Part::QUALITY_SENSOR_1:
        case Part::QUALITY_SENSOR_2:
            return BOX_HEIGHT; 
            break;
        case Part::BIN1:
        case Part::BIN2:
        case Part::BIN3:
        case Part::BIN4:
        case Part::BIN5:
        case Part::BIN6:
        case Part::BIN7:
        case Part::BIN8:
            return BIN_HEIGHT;
            break;
        default:
            ROS_WARN("surface code not recognized");
            return 0;
    }

}
*/
//assume drop-off poses specify the tray height;
//therefore, need to add part thickness for gripper clearance
double KukaBehaviorActionServer::get_dropoff_offset(Part part) {
    double offset;
    string part_name(part.name); //a C++ string
    offset = get_pickup_offset(part) + 0.006; //just pad w/ clearance to drop
    return offset;

    //obsolete below here
    if (part_name.compare("gear_part") == 0) {
        offset = get_pickup_offset(part) + 0.006; //assumes frame is at bottom of part, plus add clearance for drop
        //0.01 cm was virtually perfect
        return offset;
    }
    //piston_rod_part
    if (part_name.compare("piston_rod_part") == 0) {
        offset = get_pickup_offset(part) +
                 0.006; //assumes frame is at bottom of part; try 1cm offset; 7mm was not enough
        // 1cm correction looks very good--very small drop
        return offset;
    }
    ROS_WARN("part name not recognized");
    return 0.0; // don't recognize part, so just return zero
}

///given a part pose w/rt world, decide if the part is right-side up or up-side down
bool KukaBehaviorActionServer::eval_up_down(geometry_msgs::PoseStamped part_pose_wrt_world) {
//geometry_msgs::PoseStamped part_pose_wrt_world = part.pose;
 double qx,qy;
 qx =  part_pose_wrt_world.pose.orientation.x;
 qy = part_pose_wrt_world.pose.orientation.y;
 double sum_sqd = qx*qx+qy*qy;
 if (sum_sqd>0.5) return DOWN;
 else  return UP;
}




//given a rail displacement, compute the corresponding robot base_frame w/rt world coordinates and return as an affine3
Eigen::Affine3d KukaBehaviorActionServer::affine_base_link(double q_rail) {
    Eigen::Affine3d affine_base_link_wrt_world;
    Eigen::Quaterniond q;
    Eigen::Vector3d Oe;
    Oe[0] = BASE_LINK_X_COORD; //-0.05
    Oe[1] = q_rail+1.0;
    Oe[2] = BASE_LINK_HEIGHT; //0.700

    q.x() = 0;
    q.y() = 0;
    q.z() = 0;
    q.w() = 1;
    Eigen::Matrix3d Re(q);
    affine_base_link_wrt_world.linear() = Re;
    affine_base_link_wrt_world.translation() = Oe;
    return affine_base_link_wrt_world;
}

Eigen::Affine3d KukaBehaviorActionServer::affine_vacuum_pickup_pose_wrt_base_link(Part part, double q_rail) {
    Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link;
    Eigen::Affine3d affine_part_wrt_base_link;
    //from part, extract pose w/rt world
    geometry_msgs::PoseStamped part_pose_wrt_world = part.pose;
    Eigen::Affine3d affine_part_wrt_world, affine_base_link_wrt_world;
    
    affine_base_link_wrt_world = affine_base_link(q_rail);
    ROS_INFO("affine_vacuum_pickup_pose_wrt_base_link:");
    ROS_INFO_STREAM("using q_rail = "<<q_rail<<" origin of affine_base_link_wrt_world = "<<affine_base_link_wrt_world.translation().transpose()<<endl);
    affine_part_wrt_world = xformUtils_.transformPoseToEigenAffine3d(part_pose_wrt_world);
    ROS_INFO_STREAM("origin of part w/rt world: "<<affine_part_wrt_world.translation().transpose()<<endl);

    //manual repair of pickup height:
    //Eigen::Vector3d Oe;
    //Oe = affine_part_wrt_world.translation();

    //Oe[2]=get_surface_height(part); //assumes part frame should be flush with target surface
    //affine_part_wrt_world.translation() = Oe;

    affine_part_wrt_base_link = affine_base_link_wrt_world.inverse() * affine_part_wrt_world;
    if (!get_grasp_transform(part, grasp_transform_)) {
        ROS_WARN("did not recognize this part; using identity grasp transform");
    }

    //compute desired gripper pose from part pose and appropriate grasp transform
    //gripper_wrt_base = T_part_wrt_base*T_gripper_wrt_part
    affine_vacuum_gripper_pose_wrt_base_link = affine_part_wrt_base_link * grasp_transform_.inverse();

    //generalize this to use grasp transform!
    //double pickup_offset = get_pickup_offset(part);
    //add this to the z component of the gripper pose:
    // Eigen::Vector3d Oe;
    //Oe = affine_vacuum_gripper_pose_wrt_base_link.translation();
    //Oe[2]=get_surface_height(part)+pickup_offset-BASE_LINK_HEIGHT;
    //affine_vacuum_gripper_pose_wrt_base_link.translation() = Oe;
    return affine_vacuum_gripper_pose_wrt_base_link;
}

Eigen::Affine3d KukaBehaviorActionServer::affine_vacuum_dropoff_pose_wrt_base_link(Part part, double q_rail) {
    //from part, extract pose w/rt world
    //also, from part name, get vertical offset of grasp pose from agv-dropoff frame;
    //note: agv frame is at BOTTOM of part, but vaccum gripper must be at TOP of part
    //destination pose refers to bottom of part on AGV tray
    //have: agv1_tray_frame_wrt_world_
    Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link;
    Eigen::Affine3d affine_part_wrt_base_link, affine_part_wrt_world;
    //from part, extract pose w/rt world
    //geometry_msgs::PoseStamped part_pose_wrt_agv = part.pose; //this is presumably w/rt tray frame
    geometry_msgs::PoseStamped part_pose_wrt_world = part.pose;  //nope--w/rt world
    string frame_name(part.pose.header.frame_id);
    cout << frame_name << endl;
    ROS_INFO_STREAM("requested part pose w/rt world: " << part_pose_wrt_world);
    //ROS_INFO("part frame: %s",part.pose.header.frame_id);
    Eigen::Affine3d affine_part_wrt_tray, affine_base_link_wrt_world;
    affine_base_link_wrt_world = affine_base_link(q_rail);
    //affine_part_wrt_tray = xformUtils_.transformPoseToEigenAffine3d(part_pose_wrt_agv);
    affine_part_wrt_world = xformUtils_.transformPoseToEigenAffine3d(part_pose_wrt_world);

    //manual repair of dropoff height:
    Eigen::Vector3d Oe;
    Oe = affine_part_wrt_world.translation();

    //what is the height of the surface on which we want to place the part?
    Oe[2] = BOX_SURFACE_HT_WRT_WORLD;//get_surface_height(part); //assumes part frame should be flush with target surface
    bool part_is_up = eval_up_down(part.pose);
        //HACK ADJUSTMENT FOR INVERTED PULLEY:
    // to place an inverted pulley on a surface, its origin should be PULLEY_PART_THICKNESS above the surface;
    string part_name(part.name); //a C++ string
    if (part_name.compare("pulley_part") == 0) {
      if (!part_is_up) {
           ROS_WARN("part is inverted");
           Oe[2] += PULLEY_PART_THICKNESS+PULLEY_PART_GRASP_Z_OFFSET;
      }
    }

    affine_part_wrt_world.translation() = Oe;
    ROS_WARN("I will instead use  part dropoff pose w/rt world of: ");
    xformUtils_.printAffine(affine_part_wrt_world);

    //affine_part_wrt_world = agv1_tray_frame_wrt_world_*affine_part_wrt_tray;
    affine_part_wrt_base_link = affine_base_link_wrt_world.inverse() * affine_part_wrt_world;
    ROS_INFO("dropoff part affine w/rt base link");
    xformUtils_.printAffine(affine_part_wrt_base_link);

    if (!get_grasp_transform(part, grasp_transform_)) {
        ROS_WARN("did not recognize this part; using identity grasp transform");
    }
    affine_vacuum_gripper_pose_wrt_base_link = affine_part_wrt_base_link * grasp_transform_.inverse();
    ROS_INFO("dropoff gripper affine w/rt base link");
    xformUtils_.printAffine(affine_vacuum_gripper_pose_wrt_base_link);
    // affine_vacuum_gripper_pose_wrt_base_link= affine_part_wrt_base_link; //start here, and offset height of gripper

    // double dropoff_offset = get_dropoff_offset(part);
    //add this to the z component of the gripper pose:

    return affine_vacuum_gripper_pose_wrt_base_link;
}

//given desired affine of gripper w/rt base_link, and given an approximate jspace solution, fill in a complete 7dof soln, if possible
//return false if no IK soln, else true
bool KukaBehaviorActionServer::compute_pickup_dropoff_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd approx_jspace_pose,Eigen::VectorXd &q_vec_soln) {
    bool success = true;
    std::vector<Eigen::VectorXd> q7dof_solns;
    Eigen::VectorXd q7dof_ref,q7dof_soln;
    q7dof_ref = fwd_solver_.map827dof(approx_jspace_pose); //convert to 6dof
    int nsolns = ik_solver_.ik_solve(affine_vacuum_gripper_pose_wrt_base_link, q7dof_solns);
    std::cout << "number of IK solutions: " << nsolns << std::endl;
    //nsolns = fwd_solver_.prune_solns_by_jnt_limits(q6dof_solns);
    if (nsolns == 0) {
        ROS_WARN("NO viable IK solutions found");
        return false; // NO SOLUTIONS
    }
    ROS_INFO_STREAM("q7dof_ref: "<<q7dof_ref.transpose()<<endl);
    for (int i=0;i<nsolns;i++) {
        ROS_INFO_STREAM("soln "<<i<<": "<<q7dof_solns[i].transpose()<<endl);
    }
    q7dof_soln = fwd_solver_.select_soln_near_qnom(q7dof_solns, q7dof_ref);
 
    double q_rail = approx_jspace_pose[7];

    q_vec_soln = fwd_solver_.map728dof(q_rail, q7dof_soln);
    ROS_INFO("q8 in Kuka coords: [%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f]", q_vec_soln[0],
             q_vec_soln[1], q_vec_soln[2], q_vec_soln[3], q_vec_soln[4], q_vec_soln[5], q_vec_soln[6], q_vec_soln[7]);             
            
    return success;
}

void KukaBehaviorActionServer::adjust_box_place_limits(Eigen::Vector3d &O_dropoff_wrt_base_link) {
    O_dropoff_wrt_base_link[2]+= DROPOFF_CLEARANCE; //add a dropoff clearance
    double y = O_dropoff_wrt_base_link[1];
    double x = O_dropoff_wrt_base_link[0];

    if (y>MAX_BOX_Y_VAL) {
        O_dropoff_wrt_base_link[1]=MAX_BOX_Y_VAL;
        ROS_WARN("limiting dropoff coords to MAX_BOX_Y_VAL");
    }
    if (y<MIN_BOX_Y_VAL) {
        O_dropoff_wrt_base_link[1]=MIN_BOX_Y_VAL;
        ROS_WARN("limiting dropoff coords to MIN_BOX_Y_VAL");
    }
    if (x>MAX_BOX_X_VAL) {
        O_dropoff_wrt_base_link[0]=MAX_BOX_X_VAL;
        ROS_WARN("limiting dropoff coords to MAX_BOX_X_VAL");
    }
    if (x<MIN_BOX_X_VAL) {
        O_dropoff_wrt_base_link[0]=MIN_BOX_X_VAL;
        ROS_WARN("limiting dropoff coords to MIN_BOX_X_VAL");
    }
}

//use this function to get grasp transform from camera view
//inputs: robot joint angles, grasped part pose w/rt world (presumably from camera snapshot)
//output: A_part/gripper (grasp transform)
//e.g., hold part in approach pose; take snapshot, get joint angles, compute grasp transform, update dropoff pose
//this has been incorporated  inside recompute_pickup_dropoff_IK, below
bool KukaBehaviorActionServer::compute_grasp_transform(Eigen::Affine3d grasped_part_pose_wrt_world, Eigen::VectorXd q_vec_joint_angles_8dof, Eigen::Affine3d &affine_part_wrt_gripper) {
    Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link, affine_vacuum_gripper_pose_wrt_world;
    Eigen::Affine3d affine_base_link_wrt_world;
    Eigen::VectorXd qvec_7dof;
    qvec_7dof.resize(7);
    for (int i = 0; i < 7; i++) { //make qvec_7dof consistent w/ FK
        qvec_7dof[i] = q_vec_joint_angles_8dof[i];
    }
    double q_rail = q_vec_joint_angles_8dof[7];
    affine_base_link_wrt_world = affine_base_link(q_rail);
    //compute affine_vacuum_gripper_pose_wrt_base_link from q_vec_joint_angles
    affine_vacuum_gripper_pose_wrt_base_link = fwd_solver_.fwd_kin_solve(qvec_7dof); //fwd_kin_solve

    //A_part/gripper = inverse(A_gripper/world)*A_part/world
    affine_part_wrt_gripper = affine_vacuum_gripper_pose_wrt_base_link.inverse() * grasped_part_pose_wrt_world;
    //compute vacuum gripper  pose w/rt world: A_gripper/world = A_baselink_wrt_world *A_gripper_wrt_base_link
    affine_vacuum_gripper_pose_wrt_world = affine_base_link_wrt_world*affine_vacuum_gripper_pose_wrt_base_link;
    //finally, the answer:
    affine_part_wrt_gripper = affine_vacuum_gripper_pose_wrt_world.inverse() * grasped_part_pose_wrt_world;

}

//computes key jspace poses: 
// desired_depart_jspace_pose_ = desired_approach_jspace_pose_
// desired_grasp_dropoff_pose_ = q_vec_soln, 
bool KukaBehaviorActionServer::recompute_pickup_dropoff_IK(Eigen::Affine3d grasped_part_pose_wrt_world,
          Eigen::Affine3d desired_part_pose_wrt_world, Eigen::VectorXd q_vec_joint_angles_8dof,Eigen::VectorXd &q_vec_soln) {
    int ans;
    Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link, affine_vacuum_gripper_pose_wrt_world;
    Eigen::Affine3d affine_base_link_wrt_world, desired_affine_part_wrt_base_link;
    Eigen::Affine3d desired_affine_vacuum_gripper_pose_wrt_base_link,affine_part_wrt_gripper;
    Eigen::VectorXd qvec_7dof;
    qvec_7dof.resize(7);
    for (int i = 0; i < 7; i++) { //make qvec_7dof consistent w/ FK
        qvec_7dof[i]=q_vec_joint_angles_8dof[i];
    }
    double q_rail = q_vec_joint_angles_8dof[7];
    ROS_INFO("current q_rail = %f",q_rail);
    affine_base_link_wrt_world = affine_base_link(q_rail);
    //compute affine_vacuum_gripper_pose_wrt_base_link from q_vec_joint_angles
    affine_vacuum_gripper_pose_wrt_base_link = fwd_solver_.fwd_kin_solve(qvec_7dof); //fwd_kin_solve
  
    //compute vacuum gripper  pose w/rt world: A_gripper/world = A_baselink_wrt_world *A_gripper_wrt_base_link
    affine_vacuum_gripper_pose_wrt_world = affine_base_link_wrt_world*affine_vacuum_gripper_pose_wrt_base_link;
        //A_part/gripper = inverse(A_gripper/world)*A_part/world

    ROS_INFO("observed grasped_part_pose_wrt_world:");
    xformUtils_.printAffine(grasped_part_pose_wrt_world);    
    
    //now we have the observed grasp transform:
    affine_part_wrt_gripper = affine_vacuum_gripper_pose_wrt_world.inverse() * grasped_part_pose_wrt_world;
    ROS_INFO("observed grasp transform, affine_part_wrt_gripper:");
    xformUtils_.printAffine(affine_part_wrt_gripper);
    Eigen::Vector3d O_part_wrt_gripper = affine_part_wrt_gripper.translation();
    double part_origin_dist = O_part_wrt_gripper.norm();
    ROS_INFO("part origin is %f m from gripper origin",part_origin_dist);
    //cout<<"enter 1: ";
    //cin>>ans;
    if (part_origin_dist> GRIPPER_OFFSET_CREDIBILITY_TOLERANCE) {
        return false;
    }
    //problem here: need to  consider what WILL  be the FK when reposition d8;
    //ASSUME  have access to box_dropoff_hover_pose_ with intended d8:
    q_vec_joint_angles_8dof[7] =  box_dropoff_hover_pose_[7];
    q_rail =  q_vec_joint_angles_8dof[7];
    ROS_INFO("intended dropoff q_rail = %f",q_rail);
    affine_base_link_wrt_world = affine_base_link(q_rail);

    desired_affine_part_wrt_base_link = affine_base_link_wrt_world.inverse() * desired_part_pose_wrt_world;

    //compute desired gripper pose from part pose and appropriate grasp transform
    // A_gripper/base_link = A_part/base_link * inverse(A_part/gripper)
    desired_affine_vacuum_gripper_pose_wrt_base_link = desired_affine_part_wrt_base_link * affine_part_wrt_gripper.inverse();   
    
        //add a dropoff clearance tolerance to z value:
    Eigen::Vector3d O_dropoff_wrt_base_link = desired_affine_vacuum_gripper_pose_wrt_base_link.translation();
    adjust_box_place_limits(O_dropoff_wrt_base_link);
    desired_affine_vacuum_gripper_pose_wrt_base_link.translation() = O_dropoff_wrt_base_link;
    bool ret_val=  compute_pickup_dropoff_IK(desired_affine_vacuum_gripper_pose_wrt_base_link,q_vec_joint_angles_8dof,q_vec_soln);
    //ROS_INFO_STREAM("recomputed dropoff IK soln: "<<q_vec_soln<<endl);
    desired_grasp_dropoff_pose_ = q_vec_soln; //also, assign this to member var
    
    //careful--this call affects member vars:
    //desired_approach_jspace_pose_= desired_depart_jspace_pose_, 
    //approach_dropoff_jspace_pose_, desired_approach_depart_pose_,  desired_grasp_dropoff_pose_
    //too many synonyms!
    if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, q_vec_soln, approach_dist_,
            desired_approach_jspace_pose_)) {
        //ROS_WARN("could not compute IK soln for approach_dropoff_jspace_pose_!");
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::UNREACHABLE;
        ret_val=false;
    }
    desired_depart_jspace_pose_ = desired_approach_jspace_pose_;
    ROS_INFO_STREAM("desired_approach_jspace_pose_: " << desired_approach_jspace_pose_.transpose());   
    
    return ret_val;
}

    //in this version, do  not provide joint angles; fnc will acquire joint angles from current joint_state publication
//computes key jspace poses: desired_approach_jspace_pose_, and arg q_vec_soln

bool KukaBehaviorActionServer::recompute_pickup_dropoff_IK(Eigen::Affine3d actual_grasped_part_pose_wrt_world,Eigen::Affine3d desired_part_pose_wrt_world,
       Eigen::VectorXd &q_vec_soln) { 
   //get new joint angles:
    //get joint states:
    get_fresh_joint_states(); //update joint_state_vec_

       //ROS_INFO_STREAM("got joint states: "<<endl<<joint_state_<<endl);
       Eigen::VectorXd q_vec_joint_angles_8dof;
       q_vec_joint_angles_8dof.resize(8);
       for (int i=0;i<8;i++) {
           q_vec_joint_angles_8dof[i] = joint_state_.position[i];
       }
       //adjust d8 for computed dropoff position:
       //ASSUMES box_dropoff_hover_pose_ was previously computed!
       //better would be  to force  this as an arg
       //q_vec_joint_angles_8dof[7] = box_dropoff_hover_pose_[7];
       ROS_INFO_STREAM("got current joint angles: "<<endl<<q_vec_joint_angles_8dof.transpose()<<endl);
       bool success =  recompute_pickup_dropoff_IK(actual_grasped_part_pose_wrt_world,
          desired_part_pose_wrt_world, q_vec_joint_angles_8dof,q_vec_soln);
       ROS_INFO_STREAM("current joint angles: "<<q_vec_joint_angles_8dof.transpose()<<endl);
       ROS_INFO_STREAM("recomputed dropoff IK soln: "<<q_vec_soln.transpose()<<endl);
       if (!success) ROS_WARN("recompute_pickup_dropoff_IK was not successful");
       
       return success;
}

//get_pickup_IK(affine_vacuum_pickup_pose_wrt_base_link_, bin_hover_jspace_pose_, box_placement_location_code, dropoff_jspace_pose_))
//BROKEN!!s
/*
bool KukaBehaviorActionServer::get_pickup_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,
                                          unsigned short int box_placement_location_code, Eigen::VectorXd &q_vec_soln) {
    bool success = true;
    std::vector<Eigen::VectorXd> q7dof_solns; 
    Eigen::VectorXd q7dof_ref, q7dof_soln;
    Eigen::VectorXd q_nom;
    ROS_INFO("computing manip pose IK");
    ROS_INFO_STREAM("desired position: "<<affine_vacuum_gripper_pose_wrt_base_link.translation().transpose()<<endl);
    ROS_INFO_STREAM("desired orientation: ");
    ROS_INFO_STREAM(affine_vacuum_gripper_pose_wrt_base_link.linear());

    int nsolns = ik_solver_.ik_solve(affine_vacuum_gripper_pose_wrt_base_link, q7dof_solns);
    if (nsolns == 0) {
        ROS_WARN("NO viable IK solutions found");
        return false; // NO SOLUTIONS
    }    
    //std::cout << "number of IK solutions: " << nsolns << std::endl;
    //nsolns = fwd_solver_.prune_solns_by_jnt_limits(q6dof_solns);
    xxx FIX ME!!  need q_nom
    //if(!get_pose_from_code(box_placement_location_code, q_nom)) { //look up jspace pose from pose code
    //    ROS_WARN("box placement code for q_nom not recognized!");
    //    return false;
    //}
    //ROS_INFO_STREAM("q_nom: "<<q_nom.transpose()<<endl);
        
    q7dof_ref = fwd_solver_.map827dof(q_nom); //convert to 6dof  
    
    q7dof_soln = fwd_solver_.select_soln_near_qnom(q7dof_solns, q7dof_ref);

    double q_rail = q_nom[1];
 
    q_vec_soln = fwd_solver_.map728dof(q_rail, q7dof_soln);
    ROS_INFO("q8: [%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f]", q_vec_soln[0],
             q_vec_soln[1], q_vec_soln[2], q_vec_soln[3], q_vec_soln[4], q_vec_soln[5], q_vec_soln[6], q_vec_soln[7]);

    return success;
}
*/
/*
bool KukaBehaviorActionServer::get_pickup_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,
                                          Eigen::VectorXd approx_jspace_pose, bool use_wrist_far, Eigen::VectorXd &q_vec_soln) {
    bool success = true;
    std::vector<Eigen::VectorXd> q6dof_solns;
    Eigen::VectorXd q6dof_ref;
    q6dof_ref = fwd_solver_.map726dof(approx_jspace_pose); //convert to 6dof
    int nsolns = ik_solver_.ik_solve(affine_vacuum_gripper_pose_wrt_base_link, q6dof_solns);
    //std::cout << "number of IK solutions: " << nsolns << std::endl;
    nsolns = fwd_solver_.prune_solns_by_jnt_limits(q6dof_solns);
    if (nsolns == 0) {
        ROS_WARN("NO viable IK solutions found");
        return false; // NO SOLUTIONS
    }
    double q_rail = approx_jspace_pose[1];
    //select the solution that is closest to some reference--try q_6dof_bin6_pickup_pose
    Eigen::VectorXd q_fit;

    if (use_wrist_far) {
       q_fit = fwd_solver_.get_wrist_far_soln(q6dof_solns);
    }

    else {
      q_fit = fwd_solver_.get_wrist_near_soln(q6dof_solns);
    }
    cout << "best fit soln: " << q_fit.transpose() << endl;


    q_vec_soln = fwd_solver_.map627dof(q_rail, q_fit);
    ROS_INFO("q7: [%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f]", q_vec_soln[0],
             q_vec_soln[1], q_vec_soln[2], q_vec_soln[3], q_vec_soln[4], q_vec_soln[5], q_vec_soln[6]);

    return success;
}
*/
//function to compute an approach pose:
//specify the Eigen::Affine3d of grasp pose (pickup or dropoff); specify the IK solution to be used for this grasp pose;
//specify the approach vertical standoff distance (e.g. 5cm);
//return (via reference variable) the IK solution for this approach
//return false if no solution exists
bool KukaBehaviorActionServer::compute_approach_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,
                                                Eigen::VectorXd approx_jspace_pose, double approach_dist,
                                                Eigen::VectorXd &q_vec_soln) {
    ROS_INFO("compute_approach_IK: approach_dist = %f",approach_dist);
    bool success = true;
    std::vector<Eigen::VectorXd> q7dof_solns;
    Eigen::VectorXd q7dof_ref,q7dof_soln;
    q7dof_ref = fwd_solver_.map827dof(approx_jspace_pose); //convert to 6dof
    //compute the affine of the approach pose:
    Eigen::Matrix3d R_grasp;  //approach pose should have identical orientation
    R_grasp = affine_vacuum_gripper_pose_wrt_base_link.linear();
    Eigen::Vector3d zvec, O_grasp, O_approach;
    zvec = R_grasp.col(2); //approach pose should back off along pure z direction (typically, vertical)
    O_grasp = affine_vacuum_gripper_pose_wrt_base_link.translation();
    O_approach = O_grasp + approach_dist * zvec; //compute offset origin relative to grasp origin
    ROS_INFO_STREAM("gripper wrt base des grasp origin: "<<O_grasp.transpose()<<endl);
        ROS_INFO_STREAM("gripper wrt base des approach origin: "<<O_approach.transpose()<<endl);
        ROS_INFO_STREAM("zvec: "<<zvec.transpose()<<endl);

    Eigen::Affine3d approach_affine; //fill in components of approach affine
    approach_affine = affine_vacuum_gripper_pose_wrt_base_link;
    approach_affine.translation() = O_approach;
    //compute IK solutions for approach:

    int nsolns = ik_solver_.ik_solve(approach_affine, q7dof_solns);

    if (nsolns == 0) {
        ROS_WARN("NO viable IK solutions found for approach IK");
        return false; // NO SOLUTIONS
    }
    //select the solution that is closest to the chosen grasp IK soln    
    q7dof_soln = fwd_solver_.select_soln_near_qnom(q7dof_solns, q7dof_ref);

    double q_rail = approx_jspace_pose[7];
 
    //convert this back to a 7DOF vector, including rail pose
    q_vec_soln = fwd_solver_.map728dof(q_rail, q7dof_soln);
    ROS_INFO("q8: [%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f]", q_vec_soln[0],
             q_vec_soln[1], q_vec_soln[2], q_vec_soln[3], q_vec_soln[4], q_vec_soln[5], q_vec_soln[6], q_vec_soln[7]);

    return success;
}

//use this version for bins 4 and 5: J1_cruise = -1.57
//compute: current_bin_hover_pose_, computed_jspace_approach_, pickup_jspace_pose_, desired_approach_jspace_pose_, pickup_deeper_jspace_pose_
unsigned short int KukaBehaviorActionServer::alt_compute_bin_pickup_key_poses(inventory_msgs::Part part) {
    errorCode_ = robot_behavior_interface::RobotBehaviorResult::NO_ERROR;    

    
    transitionTrajectories_.get_hover_pose(part.location,current_hover_pose_,current_bin_hover_pose_code_);
    
    current_bin_hover_pose_ = current_hover_pose_; //synonym
    current_bin_hover_pose_[0] = -current_bin_hover_pose_[0]; //righty/lefty swap
    
    computed_jspace_approach_.resize(8);
    
    //back to useful stuff:
    geometry_msgs::PoseStamped part_pose_wrt_world = part.pose;
    Eigen::Affine3d affine_part_wrt_world = xformUtils_.transformPoseToEigenAffine3d(part_pose_wrt_world);
    Eigen::Vector3d O_part_wrt_world = affine_part_wrt_world.translation();
    ROS_INFO_STREAM("part origin w/rt world: "<<O_part_wrt_world.transpose()<<endl);
    double part_x = O_part_wrt_world[0];
    double part_y = O_part_wrt_world[1];
    //adjust pickup x and y, as necessary, to avoid hitting bin or overreach
    if(!bin_xy_is_reachable(part.location,part_x, part_y))
     {
       O_part_wrt_world[0] = part_x;
       O_part_wrt_world[1]= part_y;   
       affine_part_wrt_world.translation() = O_part_wrt_world;
       part.pose.pose.position.x = part_x; //alter part pickup info
       part.pose.pose.position.y = part_y;
    }

    if(!alt_compute_bin_hover_from_xy(part_x,part_y, computed_jspace_approach_)) {
        ROS_WARN("could not compute valid approach to bin--something wrong");
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::UNREACHABLE;
        return errorCode_;
    }
        ROS_INFO_STREAM("computed jspace approach: "<<computed_jspace_approach_.transpose()<<endl);

    //computed_jspace_approach_ contains pose estimate for  IKs
    //compute the IK for the desired pickup pose: pickup_jspace_pose_
    //first, get the equivalent desired affine of the vacuum gripper w/rt base_link;
    //need to provide the Part info and the rail displacement
    //Eigen::Affine3d RobotMoveActionServer::affine_vacuum_pickup_pose_wrt_base_link(Part part, double q_rail)
    affine_vacuum_pickup_pose_wrt_base_link_ = affine_vacuum_pickup_pose_wrt_base_link(part,
            computed_jspace_approach_[7]);
    
    //NEED THE FOLLOWING VALUE for pickup_jspace_pose_
    //provide desired gripper pose w/rt base_link, and choose soln closest to some reference jspace pose, e.g. hover pose
    //if (!get_pickup_IK(cart_grasp_pose_wrt_base_link,computed_jspace_approach_,&q_vec_soln);
    if (!compute_pickup_dropoff_IK(affine_vacuum_pickup_pose_wrt_base_link_, computed_jspace_approach_, pickup_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for pickup pose!");
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::UNREACHABLE;
        return errorCode_;
    }

    ROS_INFO_STREAM("pickup_jspace_pose_: " << pickup_jspace_pose_.transpose());

    //compute approach_pickup_jspace_pose_:  for approximate Cartesian descent and depart
    //compute_approach_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd computed_jspace_approach_,double approach_dist,Eigen::VectorXd &q_vec_soln);
    if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, pickup_jspace_pose_, approach_dist_,
            desired_approach_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for pickup approach pose!");
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::UNREACHABLE;
        return errorCode_;
    }
    if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, pickup_jspace_pose_, depart_dist_,
            desired_depart_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for depart pose!");
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::UNREACHABLE;
        return errorCode_;
    }    
    
 //   
    //desired_approach_depart_pose_ = approach_pickup_jspace_pose_;
    ROS_INFO_STREAM("desired_approach_jspace_pose_: " << desired_approach_jspace_pose_.transpose());
    //modify computed_jspace_approach_ to use same wrist orientation as soln
    /* obsolete...
    for (int i=4;i<7;i++) { current_hover_pose_[i] = desired_approach_jspace_pose_[i];
                            computed_jspace_approach_[i] = desired_approach_jspace_pose_[i];
                            computed_bin_escape_jspace_pose_[i] = desired_approach_jspace_pose_[i]; }    
    */
    if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, pickup_jspace_pose_, deep_grasp_dist_,
            pickup_deeper_jspace_pose_)) {
        ROS_WARN("could not compute IK soln for deep pickup approach pose");
        ROS_WARN("re-using grasp pose; not returning an error");
        pickup_deeper_jspace_pose_ = pickup_jspace_pose_;
        //errorCode_ = robot_behavior_interface::RobotBehaviorResult::UNREACHABLE;
        //return errorCode_;
    }
    ROS_INFO_STREAM("pickup_deeper_jspace_pose_: " << pickup_deeper_jspace_pose_.transpose());  
    
    errorCode_ = robot_behavior_interface::RobotBehaviorResult::NO_ERROR;   

    ROS_INFO("SUCCESSFUL COMPUTATION OF KEY POSES FOR BIN PICKUP");
    return errorCode_;
        
}


//use this for bins 4,5
bool KukaBehaviorActionServer::alt_compute_bin_hover_from_xy(double x_part,double y_part, Eigen::VectorXd &qvec) {
    qvec.resize(8);
    double d8,J1_ang;
    //some magic numbers: bin locations--> x_frontrow = -0.651; x_midrow = -0.776; x_backrow=-0.89
    /* already done by bin_xy_is_reachable() 
    if (x_part< MIN_BIN_X_VAL) {
        ROS_WARN("part is too far away to reach");
        x_part = MIN_BIN_X_VAL;
        ROS_INFO("recommend trying x_part = %f", x_part);
        return false;    
    }
    if (x_part> MAX_BIN_X_VAL) {
        ROS_WARN("something wrong--part is too close to front of bin");
        x_part = MAX_BIN_X_VAL;
        ROS_INFO("recommend trying x_part = %f", x_part);        
        return false;
    }
    */
    if (x_part< BIN_FAR_THRESHOLD) { //FAR--backrow case:
        //this pose has arm nearly fully outstretched and J1 rotated to face part, nearly  head-on
        //adjust slightly, so approach is partly angled, reflected from non-flipped approach
        qvec<< -0.21, -1.65, 0.0, 0.1, 0, -1.2, 0, -0.527;
        d8 = y_part -Y_BASE_WRT_WORLD_AT_D8_HOME - 0.14; //head-on approach; line up sled with part y
        qvec[7]=d8;
        return true;
    }
    //else, x is not backrow; compute values for J1_ang and d8, freezing other joints:
    double radius = 0.77; //this is radius from base to gripper at proposed arm pose
    //slightly bent elbow, suitable for access to hover over bin;
    //radius above corresponds to this pose of shoulder and elbow angles
    qvec<< -0.7, -1.35, 0, 0.6, -0.113, -0.807, 2.8085, -0.355; // left front part approach
    //center front; 0.7, -1.35, 0, 0.8, -0.113, -0.807, 2.8085, -0.18
    //consider a 2-DOF robot with one revolute and one prismatic joint;
    //J1 is the revolute joint, and the track is the prismatic joint
    //compute J1 and d8 to place the gripper over a part at give  (x,y), assuming
    //elbow and shoulder angles as chosen above  (--> radius)
    //formula: fabs(dx)/RADIUS = sin(pi/2-theta_J1) 
    // theta_J1 = pi/2 - asin(fabs(dx)/RADIUS) 
    double dx = fabs(x_part-X_BASE_WRT_WORLD); //dist from rail to part
    J1_ang = -M_PI/2.0 + asin(dx/radius);
    //AND: dy_part_wrt_link0 = RADIUS*cos(1.57-theta_J1)
    //d8 = y_part_wrt_world - fabs(dy_part_wrt_link0)
    double dy_part_wrt_link0 = radius*cos(-M_PI/2.0 +J1_ang);
    d8 = y_part -Y_BASE_WRT_WORLD_AT_D8_HOME -fabs(dy_part_wrt_link0);
    ROS_INFO("y_part = %f, dy_part_wrt_link0 = %f, d8 = %f",y_part,dy_part_wrt_link0, d8);
            
    qvec[0]=J1_ang;
    qvec[7] = d8;
    
    return true;
}

bool KukaBehaviorActionServer::get_fresh_joint_states() {
    got_new_joint_states_=false;
    for (int i=0;i<10;i++) {      
       ros::spinOnce();
       if (got_new_joint_states_) return true;
       ros::Duration(0.05).sleep();
    }
    return false; //give up after 10 tries
}

//a bunch of magic numbers for joint-space poses for the UR10
void RobotMoveActionServer::set_key_poses() {
    q_conveyor_hover_pose_.resize(7);
    q_conveyor_cruise_pose_.resize(7);
    approach_pickup_jspace_pose_.resize(7);
    

    q_box_Q2_hover_pose_.resize(7);
    q_box_Q2_cruise_pose_.resize(7);
    q_Q2_discard_pose_.resize(7);

    q_bin1_hover_pose_.resize(7);
    q_bin2_hover_pose_.resize(7);
    q_bin3_hover_pose_.resize(7);
    q_bin4_hover_pose_.resize(7);
    q_bin5_hover_pose_.resize(7);
    q_bin6_hover_pose_.resize(7);
    q_bin7_hover_pose_.resize(7);
    q_bin8_hover_pose_.resize(7);

    q_bin1_cruise_pose_.resize(7);
    q_bin2_cruise_pose_.resize(7);
    q_bin3_cruise_pose_.resize(7);
    q_bin4_cruise_pose_.resize(7);
    q_bin5_cruise_pose_.resize(7);
    q_bin6_cruise_pose_.resize(7);
    q_bin7_cruise_pose_.resize(7);
    q_bin8_cruise_pose_.resize(7);

    q_bin_pulley_flip_.resize(7);

    q_des_7dof_.resize(7);
    q_cruise_pose_.resize(7);
    bin_cruise_jspace_pose_.resize(7);
    bin_hover_jspace_pose_.resize(7);
    pickup_jspace_pose_.resize(7);
    dropoff_jspace_pose_.resize(7);
    q_bin_cruise_pose_.resize(7);  
    

    //2.14, 1.5, -1.50, 2.8, 3.27, -1.51, 0 bin5 cruise
    //1.0, 1.5, -0.8, 2.8, 4.2, -1.51, 0 bin5 hover
    //bin1 y: bin5_y - 0.81*4

    //2.14, -1.74, -1.50, 2.8, 3.27, -1.51 //bin1_cruise
    int ans;
    //cout<<"initializing bin1 cruise pose; enter 1: ";
    //cin>>ans;
    q_bin1_cruise_pose_ << 2.14, -1.74, -1.50, 2.8, 3.27, -1.51, 0;
    q_bin1_hover_pose_ << 1.0, -1.74, -0.8, 2.8, 4.2, -1.51, 0;

    q_bin2_hover_pose_ = q_bin1_hover_pose_;
    q_bin2_hover_pose_[1] = q_bin1_hover_pose_[1]+ 0.81;

    q_bin3_hover_pose_ = q_bin1_hover_pose_;
    q_bin3_hover_pose_[1] = q_bin1_hover_pose_[1]+ 2.0*0.81;

    q_bin4_hover_pose_ = q_bin1_hover_pose_;
    q_bin4_hover_pose_[1] = q_bin1_hover_pose_[1]+ 3.0*0.81;

    q_bin5_hover_pose_ = q_bin1_hover_pose_;
    q_bin5_hover_pose_[1] = q_bin1_hover_pose_[1]+ 4.0*0.81;

    //1.0, 1.5, -0.8, 2.8, 4.2, -1.51, 0
    //q_bin5_hover_pose_<<1.0, 1.5, -0.8, 2.8, 4.2, -1.51, 0;// = q_bin1_hover_pose_;
    //q_bin5_hover_pose_[1] = 1.6; //adjust: wrist hits bin; limit of sled;  q_bin1_hover_pose_[1]+ 4.0*0.81;

    //Eigen::VectorXd q_Q1_righty_discard_,q_Q1_righty_hover_,q1_Q1_righty_hover_flip_,q_Q1_righty_cruise_;
    //Eigen::VectorXd q_Q1_lefty_discard_,q_Q1_lefty_hover_,q1_Q1_lefty_hover_flip_,q_Q1_lefty_cruise_;
    q_Q1_rvrs_hover_.resize(7);
    q_Q1_rvrs_hover_flip_.resize(7);
    q_Q1_rvrs_cruise_.resize(7);
    q_Q1_rvrs_discard_.resize(7);
    
    q_Q1_fwd_discard_.resize(7);
    q_Q1_fwd_hover_.resize(7);
    q_Q1_fwd_hover_flip_.resize(7);    
    q_Q1_fwd_cruise_.resize(7);
    
    //   Eigen::VectorXd q_Q1_arm_vertical_;
    //Eigen::VectorXd q_Q1_dropoff_near_left_,q_Q1_dropoff_far_left_,q_Q1_dropoff_near_right_,q_Q1_dropoff_far_right_;
    q_Q1_arm_vertical_.resize(7);
    q_Q1_dropoff_near_right_.resize(7);
    q_Q1_dropoff_far_right_.resize(7);
    q_Q1_dropoff_near_left_.resize(7);
    q_Q1_dropoff_far_left_.resize(7);

    q_Q1_fwd_hover_ <<      1.8, 0, -0.7, 5.8, 3.6, -1.51, 0; //qual_sensor 1 location, 2018
    q_Q1_fwd_hover_flip_ << 1.8, 0, -0.7, 5.8, 0.5, 1.51, 0;    
    q_Q1_fwd_cruise_<<      2.5, 0, -1.2, 4.5, 3.27, -1.51, 0; //
    q_Q1_fwd_discard_ <<    2.5, 0, -1.2, 4.9, 3.27, -1.51, 0; //OK--prep for left-side manip
    
    q_Q1_rvrs_discard_ <<  -1.2, 0, -2.4, 4.65, 2.35, -1.57, 0;      
    q_Q1_rvrs_hover_flip_<<-1.2, 0, -2.7, 3, 5.5, 1.57, 0;
    q_Q1_rvrs_hover_<<     -1.2, 0, -2.7, 3, 2.35, -1.57, 0;      
    q_Q1_rvrs_cruise_ <<   -1.2, 0, -2.4, 4.5, 2.35, -1.57, 0; //-1.2, 0, -2.4, 4.5, 2.35, -1.57, 0

    //RIGHTY DROPOFF POSES       
    //covers cases for  left-side of box dropoff (from robot's perspective); 
    //the following are safe transitions:
    //Q1_DROPOFF_FAR_LEFT<->Q1_HOVER<->Q1_HOVER_FLIP_WRIST<->Q1_DROPOFF_NEAR_LEFT
    q_Q1_dropoff_far_left_ << 1.4, 0, -0.28, 6.0, 3.6, -1.51, 0;
    //q_box_Q1_hover_flip_wrist_pose_ << 1.8, 0, -0.7, 5.8, 0.5, 1.51, 0;
    q_Q1_dropoff_near_left_ << 1.5, 0, -0.3, 5.8, 0.4, 1.57, 0;       
    
         //----LEFTY  POSES----
     //covers cases for right-side dropoff; the following are safe transitions:
     //Q1_DROPOFF_FAR_RIGHT<->Q1_HOVER_LEFTY_FLIP<->Q1_HOVER_LEFTY<->Q1_DROPOFF_NEAR_RIGHT   
    q_Q1_dropoff_far_right_ << -1.0, 0, -2.99, 2.8, 5.6, 1.57, 0; //-1.0, 0, -2.99, 2.8, 5.6, 1.57, 0
    q_Q1_dropoff_near_right_<< -1.1, 0, -2.95, 2.7, 2.45, -1.57, 0;
    
     //safe move sequence: cruise->  Q1_CRUISE_POSE<-->Q1_ARM_VERTICAL<-->Q1_REVERSE_CRUISE<-->Q1_HOVER_LEFTY
    q_Q1_arm_vertical_<<0, 0, -1.5, 4.5, 3.27, -1.51, 0;           
             
    
    //REPEAT for Q2 poses; only need to change sled displacement
    //q_box_Q2_hover_pose_ << 1.8, -1, -0.7, 6, 3.6, -1.51, 0; //qual_sensor 1
    q_Q2_discard_pose_ << 2.5, -1, -1.2, 4.9, 3.27, -1.51, 0;
    //q_box_Q2_cruise_pose_ = q_box_Q1_cruise_pose_; //FIX ME!
    //q_box_Q2_cruise_pose_[1] = -1;
    

    //Eigen::VectorXd q_Q1_arm_vertical_;
    //Eigen::VectorXd q_Q1_dropoff_near_left_,q_Q1_dropoff_far_left_,q_Q1_dropoff_near_right_,q_Q1_dropoff_far_right_;
    //------RIGHTY POSES------



    
    //DO THE SAME FOR Q2


    //q_bin_pulley_flip_ << 1.77, 1.13, -0.68, 3.2, 4.9, -3, 0;
    //set rest of bin cruise poses:
    cruise_jspace_pose(inventory_msgs::Part::BIN2,  q_bin2_cruise_pose_);
    cruise_jspace_pose(inventory_msgs::Part::BIN3,  q_bin3_cruise_pose_);
    cruise_jspace_pose(inventory_msgs::Part::BIN4,  q_bin4_cruise_pose_);
    cruise_jspace_pose(inventory_msgs::Part::BIN5,  q_bin5_cruise_pose_);

    q_init_pose_ = q_bin3_cruise_pose_;
    approach_dist_ = 0.05; //arbitrarily set the approach offset value, e.g. 5cm

}

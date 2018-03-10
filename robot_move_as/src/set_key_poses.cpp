//a bunch of magic numbers for joint-space poses for the UR10
void RobotMoveActionServer::set_key_poses() {
    q_conveyor_hover_pose_.resize(7);
    q_conveyor_cruise_pose_.resize(7);
    approach_pickup_jspace_pose_.resize(7);
    
    q_box_Q1_hover_pose_.resize(7);
    q_box_Q1_cruise_pose_.resize(7);
    q_Q1_discard_pose_.resize(7);
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

    q_bin1_cruise_pose_ << 2.14, -1.32, -1.50, 3.14, 3.27, -1.51, 0;
    q_bin1_hover_pose_ << 1.0, -1.32, -0.8, 3.14, 4.2, -1.51, 0;

    q_bin2_hover_pose_ = q_bin1_hover_pose_;
    q_bin2_hover_pose_[1] = q_bin1_hover_pose_[1]+ 0.81;

    q_bin3_hover_pose_ = q_bin1_hover_pose_;
    q_bin3_hover_pose_[1] = q_bin1_hover_pose_[1]+ 2.0*0.81;

    q_bin4_hover_pose_ = q_bin1_hover_pose_;
    q_bin4_hover_pose_[1] = q_bin1_hover_pose_[1]+ 3.0*0.81;

    q_bin5_hover_pose_ = q_bin1_hover_pose_;
    q_bin5_hover_pose_[1] = q_bin1_hover_pose_[1]+ 4.0*0.81;

    q_box_Q1_hover_pose_ << 1.8, 0, -0.7, 6, 3.6, -1.51, 0; //qual_sensor 1 location, 2018
    q_box_Q2_hover_pose_ << 1.8, -1, -0.7, 6, 3.6, -1.51, 0; //qual_sensor 1
    q_Q1_discard_pose_ << 2.5, 0, -1.2, 4.9, 3.27, -1.51, 0;
    q_Q2_discard_pose_ << 2.5, -1, -1.2, 4.9, 3.27, -1.51, 0;
    q_box_Q1_cruise_pose_<< 2.5, 0, -1.2, 4.5, 3.27, -1.51, 0; //FIX ME!!
    q_box_Q2_cruise_pose_ = q_box_Q2_hover_pose_;


    //q_bin_pulley_flip_ << 1.77, 1.13, -0.68, 3.2, 4.9, -3, 0;

    approach_dist_ = 0.05; //arbitrarily set the approach offset value, e.g. 5cm

}

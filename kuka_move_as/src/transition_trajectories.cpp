#include <kuka_move_as/TransitionTrajectories.h>

TransitionTrajectories::TransitionTrajectories() {
  ROS_INFO("populating transition trajectories map");
  fill_transition_traj_map();
}


//fnc to make a vector of doubles from a C-style array of doubles
//provide size of C-array as an arg
//there are more general ways, but meh...
vector<double>   TransitionTrajectories::c_array_to_cpp_vec(const double c_array[],int nvals) {
  vector<double> cpp_vec;
  cpp_vec.resize(nvals);
  for (int i=0;i<nvals;i++) {
    cpp_vec[i] = c_array[i];
  }
}

void TransitionTrajectories::copy_point(const double q_array[],trajectory_msgs::JointTrajectoryPoint &trajectory_point) {
   trajectory_point.positions.clear();
   trajectory_point.positions.resize(8);
  for (int i=0;i<NDOF;i++)
     trajectory_point.positions[i] = q_array[i];
}

int TransitionTrajectories::get_trajectory(int start_code, int end_code, trajectory_msgs::JointTrajectory &transition_traj) {
  transition_traj = transition_traj_map_[start_code][end_code];
  //ROS_INFO_STREAM("transition traj: "<<endl<<transition_traj<<endl);
  int npts = transition_traj.points.size();
  return npts;
}

//concatenate two trajectories
//  trajectory_msgs::JointTrajectory concat_trajs(trajectory_msgs::JointTrajectory a, trajectory_msgs::JointTrajectory b);
trajectory_msgs::JointTrajectory TransitionTrajectories::concat_trajs(trajectory_msgs::JointTrajectory traj1, 
   trajectory_msgs::JointTrajectory traj2) {
   trajectory_msgs::JointTrajectory traj3 = traj1;
   int npts1 = traj1.points.size();
   trajectory_msgs::JointTrajectoryPoint trajectory_point;
   trajectory_point = traj1.points.back();
//trajectory_point.time_from_start = ros::Duration(2.0);
   ros::Duration traj1_arrival_time = trajectory_point.time_from_start;
   ros::Duration next_arrival_time;
   int npts2 = traj2.points.size();
   //NOTE: ASSUME endpoint of traj1 = startpoint of traj2, so skip repeating first point of traj2
   for (int i=1;i<npts2;i++) {
     trajectory_point = traj2.points[i];
     trajectory_point.time_from_start+=traj1_arrival_time;
     traj3.points.push_back(trajectory_point);
   }
   
  return traj3;
}


void TransitionTrajectories::fill_transition_traj_map() {
   trajectory_msgs::JointTrajectory transition_traj;
   trajectory_msgs::JointTrajectoryPoint trajectory_point;
   trajectory_point.positions.clear();
   trajectory_point.positions.resize(8);
   transition_traj.joint_names.clear();     
   
   transition_traj.points.clear(); // can clear components without clearing entire trajectory_msg


    //[iiwa_joint_1, iiwa_joint_2, iiwa_joint_3, iiwa_joint_4, iiwa_joint_5, iiwa_joint_6,
    //iiwa_joint_7, linear_arm_actuator_joint
    transition_traj.joint_names.push_back("iiwa_joint_1");
    transition_traj.joint_names.push_back("iiwa_joint_2");
    transition_traj.joint_names.push_back("iiwa_joint_3");
    transition_traj.joint_names.push_back("iiwa_joint_4");
    transition_traj.joint_names.push_back("iiwa_joint_5");
    transition_traj.joint_names.push_back("iiwa_joint_6");
    transition_traj.joint_names.push_back("iiwa_joint_7");
    transition_traj.joint_names.push_back("linear_arm_actuator_joint");


    //------------------ initialization: go from home pose to bin2 cruise pose= INIT pose
    transition_traj.points.clear();
    //HOME_POSE_CODE
    copy_point(HOME_POSE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);
    //HOME_POSE_ROTATED
    copy_point(HOME_POSE_ROTATED_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    //INIT_POSE_array
    copy_point(INIT_POSE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[INIT_POSE_CODE][INIT_POSE_CODE] = transition_traj;
    transition_traj_map_[HOME_POSE_CODE][INIT_POSE_CODE] = transition_traj; //synonym

    //MOVES involving BIN (near or far) and bin-cruise:

    //--------MOVES FROM BIN2 CRUISE POSE to BIN2 (near and far)--------- 
    //q_BIN2_CRUISE_POSE
    transition_traj.points.clear();
    copy_point(BIN2_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);

    //q_BIN2_HOVER_NEAR
    copy_point(BIN2_HOVER_NEAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN2_CRUISE_CODE][BIN2_HOVER_NEAR_CODE] = transition_traj;    


    //q_BIN2_HOVER_FAR
    copy_point(BIN2_HOVER_FAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN2_CRUISE_CODE][BIN2_HOVER_FAR_CODE] = transition_traj;

    //-----moves that WITHDRAW FROM q_BIN2_HOVER_FAR:
    transition_traj.points.clear();
    copy_point(BIN2_HOVER_FAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);

    //q_BIN2_HOVER_NEAR
    copy_point(BIN2_HOVER_NEAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN2_HOVER_FAR_CODE][BIN2_HOVER_NEAR_CODE] = transition_traj; 

    //BIN2_CRUISE_array
    copy_point(BIN2_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN2_HOVER_FAR_CODE][BIN2_CRUISE_CODE] = transition_traj; 

    //extra...keep going: flip over and go to Q1 HOVER via Q1 CRUISE
    copy_point(CRUISE_FLIP_MID_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(6.0);
    transition_traj.points.push_back(trajectory_point);

    copy_point(Q1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(8.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN2_HOVER_FAR_CODE][Q1_CRUISE_CODE] = transition_traj; 

    // on to hover:
    copy_point(Q1_HOVER_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(10.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN2_HOVER_FAR_CODE][Q1_HOVER_CODE] = transition_traj; 

    //------withdraw from BIN2_HOVER_NEAR---------
    transition_traj.points.clear();

    //q_BIN2_HOVER_NEAR
    copy_point(BIN2_HOVER_NEAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);

    //BIN2_CRUISE_array
    copy_point(BIN2_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN2_HOVER_NEAR_CODE][BIN2_CRUISE_CODE] = transition_traj; 

    //extra...keep going: flip over and go to Q1 HOVER via Q1 CRUISE
    copy_point(CRUISE_FLIP_MID_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);

    copy_point(Q1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(6.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN2_HOVER_NEAR_CODE][Q1_CRUISE_CODE] = transition_traj; 

    // on to hover:
    copy_point(Q1_HOVER_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(8.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN2_HOVER_NEAR_CODE][Q1_HOVER_CODE] = transition_traj; 

    //-------FLIP move, from BIN2_CRUISE to Q1_CRUISE (and on)
   transition_traj.points.clear();

    //BIN2_CRUISE_array
    copy_point(BIN2_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);

    //extra...keep going: flip over and go to Q1 HOVER via Q1 CRUISE
    copy_point(CRUISE_FLIP_MID_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);

    copy_point(Q1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN2_CRUISE_CODE][Q1_CRUISE_CODE] = transition_traj; 

    // on to hover:
    copy_point(Q1_HOVER_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(6.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN2_CRUISE_CODE][Q1_HOVER_CODE] = transition_traj; 

   //--------MOVES FROM BIN1 CRUISE POSE to BIN1 (near and far)--------- 
    //q_BIN1_CRUISE_POSE
    transition_traj.points.clear();
    copy_point(BIN1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);

    //q_BIN1_HOVER_NEAR
    copy_point(BIN1_HOVER_NEAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN1_CRUISE_CODE][BIN1_HOVER_NEAR_CODE] = transition_traj;    


    //q_BIN1_HOVER_FAR
    copy_point(BIN1_HOVER_FAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN1_CRUISE_CODE][BIN1_HOVER_FAR_CODE] = transition_traj;

    //-----moves that WITHDRAW FROM q_BIN1_HOVER_FAR:
    transition_traj.points.clear();
    copy_point(BIN1_HOVER_FAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);

    //q_BIN1_HOVER_NEAR
    copy_point(BIN1_HOVER_NEAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN1_HOVER_FAR_CODE][BIN1_HOVER_NEAR_CODE] = transition_traj; 

    //BIN2_CRUISE_array
    copy_point(BIN2_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN1_HOVER_FAR_CODE][BIN1_CRUISE_CODE] = transition_traj; 

    //extra...keep going: flip over and go to Q1 HOVER via Q1 CRUISE
    copy_point(CRUISE_FLIP_MID_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(6.0);
    transition_traj.points.push_back(trajectory_point);

    copy_point(Q1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(8.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN1_HOVER_FAR_CODE][Q1_CRUISE_CODE] = transition_traj; 

    // on to hover:
    copy_point(Q1_HOVER_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(10.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN1_HOVER_FAR_CODE][Q1_HOVER_CODE] = transition_traj; 

    //------withdraw from BIN1_HOVER_NEAR---------
    transition_traj.points.clear();

    //q_BIN1_HOVER_NEAR
    copy_point(BIN1_HOVER_NEAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);

    //BIN1_CRUISE_array
    copy_point(BIN1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN1_HOVER_NEAR_CODE][BIN1_CRUISE_CODE] = transition_traj; 

    //extra...keep going: flip over and go to Q1 HOVER via Q1 CRUISE
    copy_point(CRUISE_FLIP_MID_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);

    copy_point(Q1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(6.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN1_HOVER_NEAR_CODE][Q1_CRUISE_CODE] = transition_traj; 

    // on to hover:
    copy_point(Q1_HOVER_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(8.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN1_HOVER_NEAR_CODE][Q1_HOVER_CODE] = transition_traj; 

    //-------FLIP move, from BIN1_CRUISE to Q1_CRUISE (and on)
    transition_traj.points.clear();

    //BIN1_CRUISE_array
    copy_point(BIN1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);

    //extra...keep going: flip over and go to Q1 HOVER via Q1 CRUISE
    copy_point(CRUISE_FLIP_MID_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);

    copy_point(Q1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN1_CRUISE_CODE][Q1_CRUISE_CODE] = transition_traj; 

    // on to hover:
    copy_point(Q1_HOVER_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(6.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN1_CRUISE_CODE][Q1_HOVER_CODE] = transition_traj; 

    //--------repeat for BIN3:
  //--------MOVES FROM BIN3 CRUISE POSE to BIN3 (near and far)--------- 
    //q_BIN3_CRUISE_POSE
    transition_traj.points.clear();
    copy_point(BIN3_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);

    //q_BIN3_HOVER_NEAR
    copy_point(BIN3_HOVER_NEAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN3_CRUISE_CODE][BIN3_HOVER_NEAR_CODE] = transition_traj;    


    //q_BIN3_HOVER_FAR
    copy_point(BIN3_HOVER_FAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN3_CRUISE_CODE][BIN3_HOVER_FAR_CODE] = transition_traj;

    //-----moves that WITHDRAW FROM q_BIN3_HOVER_FAR:
    transition_traj.points.clear();
    copy_point(BIN3_HOVER_FAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);

    //q_BIN3_HOVER_NEAR
    copy_point(BIN3_HOVER_NEAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN3_HOVER_FAR_CODE][BIN3_HOVER_NEAR_CODE] = transition_traj; 

    //BIN3_CRUISE_array
    copy_point(BIN3_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN3_HOVER_FAR_CODE][BIN3_CRUISE_CODE] = transition_traj; 

    //extra...keep going: flip over and go to Q1 HOVER via Q1 CRUISE
    copy_point(CRUISE_FLIP_MID_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(6.0);
    transition_traj.points.push_back(trajectory_point);

    copy_point(Q1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(8.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN3_HOVER_FAR_CODE][Q1_CRUISE_CODE] = transition_traj; 

    // on to hover:
    copy_point(Q1_HOVER_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(10.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN3_HOVER_FAR_CODE][Q1_HOVER_CODE] = transition_traj; 

    //------withdraw from BIN3_HOVER_NEAR---------
    transition_traj.points.clear();

    //q_BIN13_HOVER_NEAR
    copy_point(BIN3_HOVER_NEAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);

    //BIN3_CRUISE_array
    copy_point(BIN3_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN3_HOVER_NEAR_CODE][BIN3_CRUISE_CODE] = transition_traj; 

    //extra...keep going: flip over and go to Q1 HOVER via Q1 CRUISE
    copy_point(CRUISE_FLIP_MID_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);

    copy_point(Q1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(6.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN3_HOVER_NEAR_CODE][Q1_CRUISE_CODE] = transition_traj; 

    // on to hover:
    copy_point(Q1_HOVER_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(8.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN3_HOVER_NEAR_CODE][Q1_HOVER_CODE] = transition_traj; 

    //-------FLIP move, from BIN3_CRUISE to Q1_CRUISE (and on)
    transition_traj.points.clear();

    //BIN3_CRUISE_array
    copy_point(BIN3_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);

    //extra...keep going: flip over and go to Q1 HOVER via Q1 CRUISE
    copy_point(CRUISE_FLIP_MID_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);

    copy_point(Q1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN3_CRUISE_CODE][Q1_CRUISE_CODE] = transition_traj; 

    // on to hover:
    copy_point(Q1_HOVER_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(6.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN3_CRUISE_CODE][Q1_HOVER_CODE] = transition_traj; 

    //--------repeat for BIN4:
  //--------MOVES FROM BIN4 CRUISE POSE to BIN4 (near and far)--------- 
    //q_BIN4_CRUISE_POSE
    transition_traj.points.clear();
    copy_point(BIN4_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);

    //q_BIN4_HOVER_NEAR
    copy_point(BIN4_HOVER_NEAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN4_CRUISE_CODE][BIN4_HOVER_NEAR_CODE] = transition_traj;    


    //q_BIN4_HOVER_FAR
    copy_point(BIN4_HOVER_FAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN4_CRUISE_CODE][BIN4_HOVER_FAR_CODE] = transition_traj;

    //-----moves that WITHDRAW FROM q_BIN4_HOVER_FAR:
    transition_traj.points.clear();
    copy_point(BIN4_HOVER_FAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);

    //q_BIN4_HOVER_NEAR
    copy_point(BIN4_HOVER_NEAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN4_HOVER_FAR_CODE][BIN4_HOVER_NEAR_CODE] = transition_traj; 

    //BIN4_CRUISE_array
    copy_point(BIN4_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN4_HOVER_FAR_CODE][BIN4_CRUISE_CODE] = transition_traj; 

    //extra...keep going: flip over and go to Q1 HOVER via Q1 CRUISE
    copy_point(CRUISE_FLIP_MID_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(6.0);
    transition_traj.points.push_back(trajectory_point);

    copy_point(Q1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(8.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN4_HOVER_FAR_CODE][Q1_CRUISE_CODE] = transition_traj; 

    // on to hover:
    copy_point(Q1_HOVER_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(10.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN4_HOVER_FAR_CODE][Q1_HOVER_CODE] = transition_traj; 

    //------withdraw from BIN4_HOVER_NEAR---------
    transition_traj.points.clear();

    //q_BIN4_HOVER_NEAR
    copy_point(BIN4_HOVER_NEAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);

    //BIN4_CRUISE_array
    copy_point(BIN4_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN4_HOVER_NEAR_CODE][BIN4_CRUISE_CODE] = transition_traj; 

    //extra...keep going: flip over and go to Q1 HOVER via Q1 CRUISE
    copy_point(CRUISE_FLIP_MID_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);

    copy_point(Q1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(6.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN4_HOVER_NEAR_CODE][Q1_CRUISE_CODE] = transition_traj; 

    // on to hover:
    copy_point(Q1_HOVER_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(8.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN4_HOVER_NEAR_CODE][Q1_HOVER_CODE] = transition_traj; 

    //-------FLIP move, from BIN3_CRUISE to Q1_CRUISE (and on)
    transition_traj.points.clear();

    //BIN4_CRUISE_array
    copy_point(BIN4_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);

    //extra...keep going: flip over and go to Q1 HOVER via Q1 CRUISE
    copy_point(CRUISE_FLIP_MID_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);

    copy_point(Q1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN4_CRUISE_CODE][Q1_CRUISE_CODE] = transition_traj; 

    // on to hover:
    copy_point(Q1_HOVER_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(6.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN4_CRUISE_CODE][Q1_HOVER_CODE] = transition_traj; 

    //--------repeat for BIN5:
  //--------MOVES FROM BIN5 CRUISE POSE to BIN5 (near and far)--------- 
    //q_BIN5_CRUISE_POSE
    transition_traj.points.clear();
    copy_point(BIN5_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);

    //q_BIN5_HOVER_NEAR
    copy_point(BIN5_HOVER_NEAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN4_CRUISE_CODE][BIN4_HOVER_NEAR_CODE] = transition_traj;    


    //q_BIN5_HOVER_FAR
    copy_point(BIN5_HOVER_FAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN5_CRUISE_CODE][BIN5_HOVER_FAR_CODE] = transition_traj;

    //-----moves that WITHDRAW FROM q_BIN4_HOVER_FAR:
    transition_traj.points.clear();
    copy_point(BIN5_HOVER_FAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);

    //q_BIN5_HOVER_NEAR
    copy_point(BIN5_HOVER_NEAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN5_HOVER_FAR_CODE][BIN5_HOVER_NEAR_CODE] = transition_traj; 

    //BIN5_CRUISE_array
    copy_point(BIN5_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN5_HOVER_FAR_CODE][BIN5_CRUISE_CODE] = transition_traj; 

    //extra...keep going: flip over and go to Q1 HOVER via Q1 CRUISE
    copy_point(CRUISE_FLIP_MID_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(6.0);
    transition_traj.points.push_back(trajectory_point);

    copy_point(Q1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(8.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN5_HOVER_FAR_CODE][Q1_CRUISE_CODE] = transition_traj; 

    // on to hover:
    copy_point(Q1_HOVER_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(10.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN5_HOVER_FAR_CODE][Q1_HOVER_CODE] = transition_traj; 

    //------withdraw from BIN5_HOVER_NEAR---------
    transition_traj.points.clear();

    //q_BIN5_HOVER_NEAR
    copy_point(BIN5_HOVER_NEAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);

    //BIN5_CRUISE_array
    copy_point(BIN5_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN5_HOVER_NEAR_CODE][BIN5_CRUISE_CODE] = transition_traj; 

    //extra...keep going: flip over and go to Q1 HOVER via Q1 CRUISE
    copy_point(CRUISE_FLIP_MID_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);

    copy_point(Q1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(6.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN5_HOVER_NEAR_CODE][Q1_CRUISE_CODE] = transition_traj; 

    // on to hover:
    copy_point(Q1_HOVER_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(8.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN5_HOVER_NEAR_CODE][Q1_HOVER_CODE] = transition_traj; 

    //-------FLIP move, from BIN3_CRUISE to Q1_CRUISE (and on)
    transition_traj.points.clear();

    //BIN4_CRUISE_array
    copy_point(BIN5_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);

    //extra...keep going: flip over and go to Q1 HOVER via Q1 CRUISE
    copy_point(CRUISE_FLIP_MID_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);

    copy_point(Q1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN5_CRUISE_CODE][Q1_CRUISE_CODE] = transition_traj; 

    // on to hover:
    copy_point(Q1_HOVER_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(6.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN5_CRUISE_CODE][Q1_HOVER_CODE] = transition_traj; 

    //---------connect bin cruise poses to Q1_cruise pose:
    //bi-directional flip, q1_cruise <->init_pose
    transition_traj.points.clear();
    //Q1_CRUISE_array
    copy_point(Q1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);
    //flip over 
    copy_point(CRUISE_FLIP_MID_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    //INIT_POSE_CODE
    copy_point(INIT_POSE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[Q1_CRUISE_CODE][INIT_POSE_CODE] = transition_traj; 

    //bi-directional flip, q1_cruise <->init_pose,  2nd direction
    transition_traj.points.clear();
    //INIT_POSE_CODE
    copy_point(INIT_POSE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);
    //flip over 
    copy_point(CRUISE_FLIP_MID_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    //Q1_CRUISE_array
    copy_point(Q1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[INIT_POSE_CODE][Q1_CRUISE_CODE] = transition_traj; 


    //bi-directional flip, q1_cruise <->Q1_cruise
    transition_traj.points.clear();
    //Q1_CRUISE_array
    copy_point(Q1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);
    //flip over 
    copy_point(CRUISE_FLIP_MID_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    //BIN2_CRUISE
    copy_point(BIN1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[Q1_CRUISE_CODE][BIN1_CRUISE_CODE] = transition_traj; 

    //bi-directional flip, q1_cruise <->bin2_cruise
    transition_traj.points.clear();
    //Q1_CRUISE_array
    copy_point(Q1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);
    //flip over 
    copy_point(CRUISE_FLIP_MID_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    //BIN2_CRUISE
    copy_point(BIN2_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[Q1_CRUISE_CODE][BIN2_CRUISE_CODE] = transition_traj; 

    //bi-directional flip, q1_cruise <->bin3_cruise
    transition_traj.points.clear();
    //Q1_CRUISE_array
    copy_point(Q1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);
    //flip over 
    copy_point(CRUISE_FLIP_MID_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    //BIN3_CRUISE
    copy_point(BIN3_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[Q1_CRUISE_CODE][BIN3_CRUISE_CODE] = transition_traj; 

    //bi-directional flip, q1_cruise <->bin4_cruise
    transition_traj.points.clear();
    //Q1_CRUISE_array
    copy_point(Q1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);
    //flip over 
    copy_point(CRUISE_FLIP_MID_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    //BIN4_CRUISE
    copy_point(BIN4_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[Q1_CRUISE_CODE][BIN4_CRUISE_CODE] = transition_traj; 

    //bi-directional flip, q1_cruise <->bin5_cruise
    transition_traj.points.clear();
    //Q1_CRUISE_array
    copy_point(Q1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);
    //flip over 
    copy_point(CRUISE_FLIP_MID_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    //BIN4_CRUISE
    copy_point(BIN5_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[Q1_CRUISE_CODE][BIN5_CRUISE_CODE] = transition_traj; 


//----------connect INIT_POSE to bin cruise poses:
    //    copy_point(INIT_POSE_array,trajectory_point);
    //bin1 <-> init_pose <-> bin1 (cruise)
    transition_traj.points.clear();
    //BIN1_CRUISE_array
    copy_point(BIN1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);

    //INIT_POSE_array
    copy_point(INIT_POSE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN1_CRUISE_CODE][INIT_POSE_CODE] = transition_traj; 

    transition_traj.points.clear();
    //INIT_POSE_array
    copy_point(INIT_POSE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);
    //BIN1_CRUISE_array
    copy_point(BIN1_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[INIT_POSE_CODE][BIN1_CRUISE_CODE] = transition_traj; 

    //bin2 to/from init_pose; these happen to be identical
    transition_traj.points.clear();
    //BIN2_CRUISE_array
    copy_point(BIN2_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);

    //INIT_POSE_array
    copy_point(INIT_POSE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN2_CRUISE_CODE][INIT_POSE_CODE] = transition_traj; 

    transition_traj.points.clear();
    //INIT_POSE_array
    copy_point(INIT_POSE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);
    //BIN2_CRUISE_array
    copy_point(BIN2_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[INIT_POSE_CODE][BIN2_CRUISE_CODE] = transition_traj; 

    //bin3 to/from init pose 
    transition_traj.points.clear();
    //BIN3_CRUISE_array
    copy_point(BIN3_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);

    //INIT_POSE_array
    copy_point(INIT_POSE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN3_CRUISE_CODE][INIT_POSE_CODE] = transition_traj; 

    transition_traj.points.clear();
    //INIT_POSE_array
    copy_point(INIT_POSE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);
    //BIN3_CRUISE_array
    copy_point(BIN3_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[INIT_POSE_CODE][BIN3_CRUISE_CODE] = transition_traj; 

    //bin4 to/from init pose
    transition_traj.points.clear();
    //BIN4_CRUISE_array
    copy_point(BIN4_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);

    //INIT_POSE_array
    copy_point(INIT_POSE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN4_CRUISE_CODE][INIT_POSE_CODE] = transition_traj; 
 
    //bin5 to/from init pose
    transition_traj.points.clear();
    //BIN5_CRUISE_array
    copy_point(BIN5_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);

    //INIT_POSE_array
    copy_point(INIT_POSE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN5_CRUISE_CODE][INIT_POSE_CODE] = transition_traj; 

    transition_traj.points.clear();
    //INIT_POSE_array
    copy_point(INIT_POSE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);

    //BIN5_CRUISE_array
    copy_point(BIN5_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[INIT_POSE_CODE][BIN5_CRUISE_CODE] = transition_traj; 

    //extras...
    //bin2<->bin5<->bin2
    transition_traj.points.clear();
    //BIN2_CRUISE_array
    copy_point(BIN2_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);

    //BIN5_CRUISE_array
    copy_point(BIN5_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN2_CRUISE_CODE][BIN5_CRUISE_CODE] = transition_traj; 


    transition_traj.points.clear();
    //BIN5_CRUISE_array
    copy_point(BIN5_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(0.5);
    transition_traj.points.push_back(trajectory_point);

    //BIN2_CRUISE_array
    copy_point(BIN2_CRUISE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[BIN5_CRUISE_CODE][BIN2_CRUISE_CODE] = transition_traj; 

   ROS_INFO("done populating map");
}

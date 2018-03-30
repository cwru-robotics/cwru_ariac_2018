//convenient for initializing data: C-style array 
const int NJNTS = 8;
const double Q1_HOVER_array[NJNTS] = { 0.0, 1.1, 0.0, -1.6, 0, 0.45, 0, -0.35}; 
const double Q1_CRUISE_array[NJNTS] = {1.57, 1.1, 0.0, -1.6, 0, 0.45, 0, -0.35}; 
const double CRUISE_FLIP_MID_array[NJNTS] = {1.57, 1.1, 0.0, -1.6, 0, 0.45, 0, -0.35};    
const double BIN2_CRUISE_POSE_array[NJNTS] = {1.57, -1.35, 0.0, 0.5, 0, -1.2, 0, 0}; 
const double BIN2_HOVER_LEFT_NEAR_array[NJNTS] = {0.5, -1.35, 0.0, 0.5, 0, -1.2, 0, -0.4}; 
const double BIN2_HOVER_LEFT_FAR_array[NJNTS] = {0.1, -1.35, 0.0, 0.3, 0, -1.2, 0, -0.7}; 

std::map<int,std::map<int,trajectory_msgs::JointTrajectory>> transition_traj_map_;

//these just for shorthand reference to goal-message vals
const int Q1_HOVER_CODE = robot_move_as::RobotMoveGoal::Q1_HOVER_CODE;
const int Q1_CRUISE_CODE = robot_move_as::RobotMoveGoal::Q1_CRUISE_CODE;
const int CRUISE_FLIP_MID_CODE = robot_move_as::RobotMoveGoal::CRUISE_FLIP_MID_CODE;
const int BIN2_CRUISE_POSE_CODE = robot_move_as::RobotMoveGoal::BIN2_CRUISE_POSE_CODE;
const int BIN2_HOVER_LEFT_NEAR_CODE = robot_move_as::RobotMoveGoal::BIN2_HOVER_LEFT_NEAR_CODE;
const int BIN2_HOVER_LEFT_FAR_CODE = robot_move_as::RobotMoveGoal::BIN2_HOVER_LEFT_FAR_CODE;

//provide two location-code  indicies, and get back a trajectory to move from corresponding start to goal poses
bool get_trajectory(unsigned  short int start_code, unsigned short int end_code,
  trajectory_msgs::JointTrajectory &transition_traj) {
    int i_from = start_code;
    int j_to = end_code;
    //separate array of bools indicates if desired transition exists in the array (which is not fully populated)
    if (!transition_traj_exists[start_code][end_code]) {
     ROS_WARN("transition from %d to %d is not pre-computed", i_from,j_to);
     return false;
    }
    //else, trajectory has been pre-computed; return it by filling in reference var
    //merely look up the traj from the transition matrix
    transition_traj = transition_traj_matrix_[i_from][j_to];
    return true;
}

//alt: init a Eigen-style vecs, then convert to C++ vecs
void  evec_to_cppvec(Eigen::VectorXd q_vec,vector<double> &cpp_vec) {
    int npts = q_vec.size();
    cpp_vec.resize(npts);
    for (int i=0;i<npts;i++) {
      cpp_vec[i] = q_vec[i];
    }
}

//fnc to make a vector of doubles from a C-style array of doubles
//provide size of C-array as an arg
//there are more general ways, but meh...
vector<double> cpp_vec  c_array_to_cpp_vec(double c_array,int nvals) {
  vector<double> cpp_vec;
  cpp_vec.resize(nvalls);
  for (int i=0;i<nvals;i++) {
    cpp_vec[i] = c_array[i];
  }
}



//make these member vars?
//double vec_CRUISE_FLIP_MID[8],vec_BIN2_CRUISE_POSE[8], vec_BIN2_HOVER_LEFT_NEAR[8];
//double vec_Q1_CRUISE[8],vec_Q1_HOVER[8],vec_BIN2_HOVER_LEFT_FAR[8];

void set_key_poses{

}

void fill_transition_traj_map() {
   trajectory_msgs::JointTrajectory transition_traj;
   trajectory_msgs::JointTrajectoryPoint trajectory_point;
   trajectory_point1.positions.clear();
   trajectory_point1.positions.resize(8);
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


    //CRUISE_FLIP_MID
    trajectory_point.positions = c_array_to_cpp_vec(CRUISE_FLIP_MID_array,NJNTS);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    
    //q_BIN2_CRUISE_POSE
    trajectory_point.positions = c_array_to_cpp_vec(BIN2_CRUISE_POSE_array,NJNTS);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);

    //q_BIN2_HOVER_LEFT_NEAR
    trajectory_point.positions = c_array_to_cpp_vec(BIN2_HOVER_LEFT_NEAR_array,NJNTS);
    trajectory_point.time_from_start = ros::Duration(6.0);
    transition_traj.points.push_back(trajectory_point);

    //q_BIN2_HOVER_LEFT_FAR
    trajectory_point.positions = c_array_to_cpp_vec(BIN2_HOVER_LEFT_FAR_array,NJNTS);
    trajectory_point.time_from_start = ros::Duration(8.0);
    transition_traj.points.push_back(trajectory_point);

    //done computing trajectory from CRUISE_FLIP_MID to BIN2_HOVER_LEFT_FAR; install it in the 2-D map:
    transition_traj_map_[CRUISE_FLIP_MID_CODE][BIN2_HOVER_LEFT_FAR_CODE] = des_trajectory;

    //prepare to compute another transition trajectory:
    transition_traj.points.clear();

    //put traj in goal message
    //robot_goal.trajectory = des_trajectory;
}

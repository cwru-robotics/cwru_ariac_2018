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

    //CRUISE_FLIP_MID
    copy_point(CRUISE_FLIP_MID_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    //q_BIN2_CRUISE_POSE
    copy_point(BIN2_CRUISE_POSE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);

    //q_BIN2_HOVER_LEFT_NEAR
    copy_point(BIN2_HOVER_LEFT_NEAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(6.0);
    transition_traj.points.push_back(trajectory_point);

    //q_BIN2_HOVER_LEFT_FAR
    copy_point(BIN2_HOVER_LEFT_FAR_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(8.0);
    transition_traj.points.push_back(trajectory_point);

    ROS_INFO("created a traj; put it in map: ");

    //done computing trajectory from CRUISE_FLIP_MID to BIN2_HOVER_LEFT_FAR; install it in the 2-D map:
    transition_traj_map_[CRUISE_FLIP_MID_CODE][BIN2_HOVER_LEFT_FAR_CODE] = transition_traj;

    //------------------prepare to compute another transition trajectory:
    transition_traj.points.clear();
    //fix this--pretty odd init
    //CRUISE_FLIP_MID
    copy_point(CRUISE_FLIP_MID_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(2.0);
    transition_traj.points.push_back(trajectory_point);
    //q_BIN2_CRUISE_POSE
    copy_point(BIN2_CRUISE_POSE_array,trajectory_point);
    trajectory_point.time_from_start = ros::Duration(4.0);
    transition_traj.points.push_back(trajectory_point);
    transition_traj_map_[INIT_POSE_CODE][INIT_POSE_CODE] = transition_traj;

   ROS_INFO("done populating map");
}

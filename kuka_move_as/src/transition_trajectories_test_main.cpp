//test main for transitionTrajectories library:

#include <kuka_move_as/TransitionTrajectories.h>

int main(int argc, char ** argv) {
  //ros::init(argc, argv, "box_inspector");
  //ros::NodeHandle nh;

  TransitionTrajectories transitionTrajectories;
  trajectory_msgs::JointTrajectory transition_traj;
  int npts = transitionTrajectories.get_trajectory(CRUISE_FLIP_MID_CODE,BIN2_HOVER_LEFT_FAR_CODE, transition_traj);
  if (npts) {
     ROS_INFO_STREAM(transition_traj);
  }
  else {
   ROS_WARN("precomputed traj does not exist!");
  }

  if (transitionTrajectories.get_trajectory(Q1_HOVER_CODE,BIN2_HOVER_LEFT_FAR_CODE, transition_traj)) {
     ROS_INFO_STREAM(transition_traj);
  }
  else {
   ROS_WARN("precomputed traj does not exist!");
  }

  if (transitionTrajectories.get_trajectory(INIT_POSE_CODE,INIT_POSE_CODE, transition_traj)) {
     ROS_INFO_STREAM(transition_traj);
  }
  else {
   ROS_WARN("precomputed traj does not exist!");
  }

}

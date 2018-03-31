//test main for transitionTrajectories library:

#include <kuka_move_as/TransitionTrajectories.h>

int main(int argc, char ** argv) {
  //ros::init(argc, argv, "box_inspector");
  //ros::NodeHandle nh;

  TransitionTrajectories transitionTrajectories;
  trajectory_msgs::JointTrajectory transition_traj,concatenated_traj,traj1, traj2;
  int npts = transitionTrajectories.get_trajectory(CRUISE_FLIP_MID_CODE,BIN2_HOVER_FAR_CODE, transition_traj);
  if (npts) {
     ROS_INFO_STREAM(transition_traj);
  }
  else {
   ROS_WARN("precomputed traj does not exist!");
  }

  if (transitionTrajectories.get_trajectory(Q1_HOVER_CODE,BIN2_HOVER_FAR_CODE, transition_traj)) {
     ROS_INFO_STREAM(transition_traj);
  }
  else {
   ROS_WARN("precomputed traj does not exist!");
  }

  if (transitionTrajectories.get_trajectory(HOME_POSE_CODE,INIT_POSE_CODE, transition_traj)) {
     ROS_INFO_STREAM(transition_traj);
  }
  else {
   ROS_WARN("precomputed traj does not exist!");
  }

  transitionTrajectories.get_trajectory(HOME_POSE_CODE,INIT_POSE_CODE, traj1);
  ROS_INFO_STREAM("traj1: "<<endl<<traj1<<endl);

  if (transitionTrajectories.get_trajectory(BIN2_CRUISE_CODE,BIN2_HOVER_NEAR_CODE, traj2)) {
     ROS_INFO_STREAM("traj2: "<<traj2<<endl);
  }
  else {
   ROS_WARN("precomputed traj does not exist!");
  }


  concatenated_traj = transitionTrajectories.concat_trajs(traj1, traj2);
  ROS_INFO_STREAM("concat trajs"<<endl<<concatenated_traj<<endl);


}

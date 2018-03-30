//transition_trajectories class:
#ifndef TRANSITION_TRAJECTORIES_H_
#define TRANSITION_TRAJECTORIES_H_
#include <map>
#include <string>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
using namespace std;

//key poses, hard coded:
//convenient for initializing data: C-style array 
const int NDOF = 8;
const double Q1_HOVER_array[NDOF] = { 0.0, 1.1, 0.0, -1.6, 0, 0.45, 0, -0.35}; 
const double Q1_CRUISE_array[NDOF] = {1.57, 1.1, 0.0, -1.6, 0, 0.45, 0, -0.35}; 
const double CRUISE_FLIP_MID_array[NDOF] = {1.57, 1.1, 0.0, -1.6, 0, 0.45, 0, -0.35};    
const double BIN2_CRUISE_POSE_array[NDOF] = {1.57, -1.35, 0.0, 0.5, 0, -1.2, 0, 0}; 
const double BIN2_HOVER_LEFT_NEAR_array[NDOF] = {0.5, -1.35, 0.0, 0.5, 0, -1.2, 0, -0.4}; 
const double BIN2_HOVER_LEFT_FAR_array[NDOF] = {0.1, -1.35, 0.0, 0.3, 0, -1.2, 0, -0.7}; 
const double INIT_POSE_array[NDOF] = {1.57, -1.35, 0.0, 0.5, 0, -1.2, 0, 0}; 

//location codes, hard coded:
//these just for shorthand reference to goal-message vals
//FIX THESE!
const int Q1_HOVER_CODE = 1; //kuka_move_as::RobotBehaviorGoal::Q1_HOVER_CODE;
const int Q1_CRUISE_CODE = 2; //kuka_move_as::RobotBehaviorGoal::Q1_CRUISE_CODE;
const int CRUISE_FLIP_MID_CODE = 3; //kuka_move_as::RobotBehaviorGoal::CRUISE_FLIP_MID_CODE;
const int BIN2_CRUISE_POSE_CODE = 4; //kuka_move_as::RobotBehaviorGoal::BIN2_CRUISE_POSE_CODE;
const int BIN2_HOVER_LEFT_NEAR_CODE = 5; //kuka_move_as::RobotBehaviorGoal::BIN2_HOVER_LEFT_NEAR_CODE;
const int BIN2_HOVER_LEFT_FAR_CODE = 6; //kuka_move_as::RobotBehaviorGoal::BIN2_HOVER_LEFT_FAR_CODE;
const int INIT_POSE_CODE = 7; //
class TransitionTrajectories
{
  public:
   TransitionTrajectories();
  //look up a trajectory.  rtn val is number of pts in traj; specify start and end location codes
  int get_trajectory(int start_code, int end_code, trajectory_msgs::JointTrajectory &transition_traj);
  vector<double>  c_array_to_cpp_vec(const double c_array[],int nvals);
  private:
   void fill_transition_traj_map();
   void copy_point(const double q_array[],trajectory_msgs::JointTrajectoryPoint &trajectory_point);
   std::map<int,std::map<int,trajectory_msgs::JointTrajectory>> transition_traj_map_;

};


#endif

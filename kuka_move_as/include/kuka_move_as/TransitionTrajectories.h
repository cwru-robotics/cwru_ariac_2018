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

const double BIN1_CRUISE_array[NDOF] = {1.57, -1.35, 0.0, 0.5, 0, -1.2, 0, -0.81}; 
const double BIN1_HOVER_NEAR_array[NDOF] = {0.5, -1.35, 0.0, 0.5, 0, -1.2, 0, -1.21}; 
const double BIN1_HOVER_FAR_array[NDOF] = {0.1, -1.35, 0.0, 0.3, 0, -1.2, 0, -1.51}; 
  
const double BIN2_CRUISE_array[NDOF] = {1.57, -1.35, 0.0, 0.5, 0, -1.2, 0, 0}; 
const double BIN2_HOVER_NEAR_array[NDOF] = {0.5, -1.35, 0.0, 0.5, 0, -1.2, 0, -0.4}; 
const double BIN2_HOVER_FAR_array[NDOF] = {0.1, -1.35, 0.0, 0.3, 0, -1.2, 0, -0.7}; 

const double BIN3_CRUISE_array[NDOF] = {1.57, -1.35, 0.0, 0.5, 0, -1.2, 0, 0.81}; 
const double BIN3_HOVER_NEAR_array[NDOF] = {0.5, -1.35, 0.0, 0.5, 0, -1.2, 0, 0.41}; 
const double BIN3_HOVER_FAR_array[NDOF] = {0.1, -1.35, 0.0, 0.3, 0, -1.2, 0, 0.11}; 

const double BIN4_CRUISE_array[NDOF] = {1.57, -1.35, 0.0, 0.5, 0, -1.2, 0, 1.62}; 
const double BIN4_HOVER_NEAR_array[NDOF] = {0.5, -1.35, 0.0, 0.5, 0, -1.2, 0, 1.22}; 
const double BIN4_HOVER_FAR_array[NDOF] = {0.1, -1.35, 0.0, 0.3, 0, -1.2, 0, 0.92}; 

const double BIN5_CRUISE_array[NDOF] = {1.57, -1.35, 0.0, 0.5, 0, -1.2, 0, 2.43}; 
const double BIN5_HOVER_NEAR_array[NDOF] = {0.5, -1.35, 0.0, 0.5, 0, -1.2, 0, 2.03}; 
const double BIN5_HOVER_FAR_array[NDOF] = {0.1, -1.35, 0.0, 0.3, 0, -1.2, 0, 1.73}; 

const double INIT_POSE_array[NDOF] = {1.57, -1.35, 0.0, 0.5, 0, -1.2, 0, 0}; 
const double HOME_POSE_array[NDOF] = {0,0,0,0,0,0, 0, 0}; 
const double HOME_POSE_ROTATED_array[NDOF] = {1.57, 0,0,0,0,0,0,0}; 

//location codes, hard coded:
const int Q1_HOVER_CODE = 1; //kuka_move_as::RobotBehaviorGoal::Q1_HOVER_CODE;
const int Q1_CRUISE_CODE = 2; //kuka_move_as::RobotBehaviorGoal::Q1_CRUISE_CODE;
const int Q1_DISCARD_CODE = 3;

const int CRUISE_FLIP_MID_CODE = 4; //kuka_move_as::RobotBehaviorGoal::CRUISE_FLIP_MID_CODE;

const int Q2_CRUISE_CODE = 6;
const int Q2_DISCARD_CODE =7;
const int Q2_HOVER_CODE = 8;

const int HOME_POSE_CODE = 0;
const int INIT_POSE_CODE = 9; //


const int BIN1_CRUISE_CODE = 10;
const int BIN1_HOVER_NEAR_CODE = 11;
const int BIN1_HOVER_FAR_CODE = 12;

const int BIN2_CRUISE_CODE = 20; //kuka_move_as::RobotBehaviorGoal::BIN2_CRUISE_CODE;
const int BIN2_HOVER_NEAR_CODE = 21; //kuka_move_as::RobotBehaviorGoal::BIN2_HOVER_NEAR_CODE;
const int BIN2_HOVER_FAR_CODE = 22; //kuka_move_as::RobotBehaviorGoal::BIN2_HOVER_FAR_CODE;

const int BIN3_CRUISE_CODE = 30;
const int BIN3_HOVER_NEAR_CODE = 31;
const int BIN3_HOVER_FAR_CODE = 32;

const int BIN4_CRUISE_CODE = 40;
const int BIN4_HOVER_NEAR_CODE = 41;
const int BIN4_HOVER_FAR_CODE = 42;

const int BIN5_CRUISE_CODE = 50;
const int BIN5_HOVER_NEAR_CODE = 51;
const int BIN5_HOVER_FAR_CODE = 52;

class TransitionTrajectories
{
  public:
   TransitionTrajectories();
  //look up a trajectory.  rtn val is number of pts in traj; specify start and end location codes
  int get_trajectory(int start_code, int end_code, trajectory_msgs::JointTrajectory &transition_traj);
  vector<double>  c_array_to_cpp_vec(const double c_array[],int nvals);
  trajectory_msgs::JointTrajectory concat_trajs(trajectory_msgs::JointTrajectory a, trajectory_msgs::JointTrajectory b);
  private:
   void fill_transition_traj_map();
   void copy_point(const double q_array[],trajectory_msgs::JointTrajectoryPoint &trajectory_point);
   std::map<int,std::map<int,trajectory_msgs::JointTrajectory>> transition_traj_map_;

};


#endif

//transition_trajectories class:
#ifndef TRANSITION_TRAJECTORIES_H_
#define TRANSITION_TRAJECTORIES_H_
#include <map>
#include <string>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <inventory_msgs/Part.h>
using namespace std;

//key poses, hard coded:
//convenient for initializing data: C-style array 
const int NDOF = 8;
//const double Q1_HOVER_array[NDOF] = {0.0, 0.9, 0.0, -1.6, 0, 0.45, 0, -0.385}; 
const double Q1_HOVER_array[NDOF] = {-2.9, -1.5, 0, 0.4, -0.04, -1.25, 0, -0.9}; 

//0.3, 1.3, 0.0, -1.0, 0, 0.45, 0, -0.385
const double Q1_CRUISE_array[NDOF] = {1.57, -1.5, 0, 0.4, -0.04, -1.25, 0, -0.385}; 
const double CRUISE_FLIP_MID_array[NDOF] = {1.57, 1.1, 0.0, -1.6, 0, 0.45, 0, -0.385};  
const double Q1_DISCARD_array[NDOF] = {1.57, 1.1, 0.0, -1.6, 0, 0.45, 0, 1.75};  

//FIX ME!
const double Q2_HOVER_array[NDOF] = { 0.0, 1.1, 0.0, -1.6, 0, 0.45, 0, -0.8}; 
const double Q2_CRUISE_array[NDOF] = {1.57, 1.1, 0.0, -1.6, 0, 0.45, 0, -0.8}; 

////1.57, -1.35, 0.0, 0.5, 3, 0.8, 0, -0.81
//const double BIN1_CRUISE_array[NDOF] = {1.57, -1.35, 0.0, 0.5, 0, -1.2, 0, -0.81}; 
const double BIN1_CRUISE_array[NDOF] = {1.57, -1.35, 0.0, 0.5, 3, 0.8, 0, -0.81};  //note positive wrist bend
const double BIN1_NEAR_CRUISE_array[NDOF] =       {1.57, -1.35, 0, 0.5,  2.9,  1.3,  0, -0.81}; 
//const double BIN1_CENTER_ROW_CRUISE_array[NDOF] = {1.57, -1.36, 0, 0.6,  2.9,  1.3,  0, -0.88};
//1.569, -1.359, 0.005, 0.6, 2.914, 1.3, 0, -0.88
//1.564, -1.345, 0, 0.3, 2.93, 0.78, 0.387, -0.8
const double BIN1_CENTER_ROW_CRUISE_array[NDOF] = {1.57, -1.35, 0, 0.3, 2.93, 1.4, 0.387, -0.8}; //note positive wrist bend

const double BIN1_CENTER_ROW_VIA_1[NDOF] =        {0.8, -1.36, 0, 0.6,   2.9,  1.3, 0, -0.88};
const double BIN1_CENTER_ROW_VIA_2[NDOF] =        {0.5, -1.36, 0, 0.6,   2.9,  1.3, 0, -1.05};

//const double BIN1_DEEP_CRUISE_array[NDOF] =       {1.57, -1.36, 0, 0.6, -0.1, -0.95, 0, -0.88};
const double BIN1_DEEP_VIA_1[NDOF] = {0.8, -1.36, 0, 0.6, -0.1, -0.95, 0, -0.88};
const double BIN1_DEEP_VIA_2[NDOF] = {0.5, -1.36, 0, 0.6, -0.1, -0.95, 0, -1.05};
//use outstretched arm:
const double BIN1_DEEP_CRUISE_array[NDOF] = {1.57, -1.35, 0, 0.3, -0.04, -1.4, 0, -0.8};  //note negative wrist bend

//const double BIN1_HOVER_NEAR_array[NDOF] = {0.5, -1.35, 0.0, 0.5, 0, -1.2, 0, -1.21}; 
const double BIN1_HOVER_NEAR_array[NDOF] = {0.5, -1.35, 0.0, 0.5, 3, 0.8, 0, -1.21}; 
//const double BIN1_HOVER_FAR_array[NDOF] = {0.1, -1.35, 0.0, 0.3, 0, -1.2, 0, -1.51}; 
const double BIN1_HOVER_FAR_array[NDOF] = {0.1, -1.35, 0.0, 0.3, 3, 0.8, 0, -1.51}; 

  
const double BIN2_CRUISE_array[NDOF] = {1.57, -1.35, 0.0, 0.5, 0, -1.2, 0, 0}; 
const double BIN2_HOVER_NEAR_array[NDOF] = {0.5, -1.35, 0.0, 0.5, 0, -1.2, 0, -0.4}; 
const double BIN2_HOVER_FAR_array[NDOF] = {0.1, -1.35, 0.0, 0.3, 0, -1.2, 0, -0.7}; 

const double BIN3_CRUISE_array[NDOF] = {1.57, -1.35, 0.0, 0.5, 0, -1.2, 0, 0.81}; 
const double BIN3_HOVER_NEAR_array[NDOF] = {0.5, -1.35, 0.0, 0.5, 0, -1.2, 0, 0.41}; 
const double BIN3_HOVER_FAR_array[NDOF] = {0.1, -1.35, 0.0, 0.3, 0, -1.2, 0, 0.11}; 

//const double BIN4_CRUISE_array[NDOF] = {1.57, -1.35, 0.0, 0.5, 0, -1.2, 0, 1.62}; 
const double BIN4_CRUISE_array[NDOF] = {1.57, -1.35, 0.0, 0.5, 0, -1.2, 0, 1.42}; 
//const double BIN4_HOVER_NEAR_array[NDOF] = {0.5, -1.35, 0.0, 0.5, 0, -1.2, 0, 1.22}; 
const double BIN4_HOVER_NEAR_array[NDOF] = {0.5, -1.35, 0, 0.5, -0.12, -0.913, 3.013, 1.376}; 
const double BIN4_HOVER_FAR_array[NDOF] = {0.1, -1.35, 0.0, 0.3, 0, -1.2, 0, 0.92}; 

//max sled val = 1.79; careful--bin5 approach NOT  flipped
//position: [2.870471561614764, 1.3903678520995415, -0.015429008846624015, -0.7447154011426811, -2.932944045506636, -0.8267392468382049, -0.3143643136622174, 1.4464382879099866, 0.0]
//2.6260961309306126, 1.3522146683623353, 0.0018241288588187743, -0.5539856796579077, -2.9306088904300918, -0.8098751304684004, -2.6008835359030664, 1.2805203298247987
//const double BIN5_CRUISE_array[NDOF] = {1.57, 1.35, 0.0, -0.5, 0, 0.45, 0, 1.0}; 
const double BIN5_CRUISE_array[NDOF] = {1.57, 1.35, 0.0, -0.5, -0.8, -2.6, 0, 1.0}; 

const double BIN5_HOVER_NEAR_array[NDOF] = {2.64, 1.35, 0.0, -0.5, 0, 0.45, 0, 1.3}; 
const double BIN5_HOVER_FAR_array[NDOF] = {2.93, 1.35, 0.0, -0.3, 0, 0.45, 0, 1.3}; 
const double BIN5_ESCAPE_array[NDOF] = {2.3, 1.35, 0, -0.6, 0.18, 1.0, -1.0, 1.1}; 

const double INIT_POSE_array[NDOF] = {1.57, -1.35, 0.0, 0.5, 0, -1.2, 0, 0}; 
const double NOM_BIN_CRUISE_array[NDOF] = {1.57, -1.35, 0.0, 0.5, 0, -1.2, 0, 0}; 
const double HOME_POSE_array[NDOF] = {0,0,0,0,0,0, 0, 0}; 
const double HOME_POSE_ROTATED_array[NDOF] = {1.57, 0,0,0,0,0,0,0}; 

//location codes, hard coded:



//the following 3 refer to most-recently computed approach/depart pose,
//  grasp/place pose, or a grasp pose deliberately sunken into the  part, to assist grasp
const int APPROACH_DEPART_CODE = 1;
const int GRASP_PLACE_CODE = 2;
const int GRASP_DEEPER_CODE = 3;

const int HOME_POSE_CODE = 4;
const int CRUISE_FLIP_MID_CODE = 5; //kuka_move_as::RobotBehaviorGoal::CRUISE_FLIP_MID_CODE;
const int CURRENT_HOVER_CODE = 6;
const int CUSTOM_JSPACE_POSE = 7; 
const int INIT_POSE_CODE = 9; //
const int NOM_BIN_CRUISE = 9; //synonym


const int BIN1_CRUISE_CODE = 10;
const int BIN1_HOVER_NEAR_CODE = 11;
const int BIN1_HOVER_FAR_CODE = 12;

const int BIN2_CRUISE_CODE = 9;  //same as nom bin cruise code
const int BIN2_HOVER_NEAR_CODE = 21; 
const int BIN2_HOVER_FAR_CODE = 22; 

const int BIN3_CRUISE_CODE = 30;
const int BIN3_HOVER_NEAR_CODE = 31;
const int BIN3_HOVER_FAR_CODE = 32;

const int BIN4_CRUISE_CODE = 40;
const int BIN4_HOVER_NEAR_CODE = 41;
const int BIN4_HOVER_FAR_CODE = 42;

const int BIN5_CRUISE_CODE = 50;
const int BIN5_HOVER_NEAR_CODE = 51;
const int BIN5_HOVER_FAR_CODE = 52;



const int Q1_HOVER_CODE = 101; 
const int Q1_CRUISE_CODE = 102; 
const int Q1_DISCARD_CODE = 103;

const int Q2_HOVER_CODE = 201; 
const int Q2_CRUISE_CODE = 202; 
const int Q2_DISCARD_CODE = 203;


class TransitionTrajectories
{
  public:
   TransitionTrajectories();
  //look up a trajectory.  rtn val is number of pts in traj; specify start and end location codes
  int get_trajectory(int start_code, int end_code, trajectory_msgs::JointTrajectory &transition_traj);
  vector<double>  c_array_to_cpp_vec(const double c_array[],int nvals);
  trajectory_msgs::JointTrajectory concat_trajs(trajectory_msgs::JointTrajectory a,trajectory_msgs::JointTrajectory b);
   void copy_point(const double q_array[],trajectory_msgs::JointTrajectoryPoint &trajectory_point);
  //look up cruise pose by location code
  bool get_cruise_pose(unsigned short int location_code, Eigen::VectorXd &q_vec, int &pose_code);
  bool get_hover_pose(unsigned short int location_code, Eigen::VectorXd &q_vec, int &pose_code);
  void c_array_to_qvec(const double array[],Eigen::VectorXd &q_vec);
  private:
   void fill_transition_traj_map();

   std::map<int,std::map<int,trajectory_msgs::JointTrajectory>> transition_traj_map_;

};


#endif

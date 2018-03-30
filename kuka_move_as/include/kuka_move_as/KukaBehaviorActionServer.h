//
// Created by shipei on 2/17/17.
// modified wsn 3/5/18
//

#ifndef ROBOT_MOVE_AS_ROBOT_MOVE_AS_H
#define ROBOT_MOVE_AS_ROBOT_MOVE_AS_H

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <unordered_map>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <kuka_move_as/RobotBehaviorAction.h>
#include <kuka_move_as/part_dimensions.h>
#include <kuka_move_as/TransitionTrajectories.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tf/transform_listener.h>
#include <xform_utils/xform_utils.h>
#include <kuka_fk_ik/kuka_kin.h>

//#include <sensor_msgs/JointState.h>
//#include <sensor_msgs/LaserScan.h>
//#include <kuka_move_as/RobotBehaviorInterface.h>

#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>
#include <inventory_msgs/Part.h>



//#include <robot_move_as/RobotBehaviorInterface.h>

using namespace std;
using namespace Eigen;
using namespace inventory_msgs;

const bool UP = true;
const bool DOWN = false;


class KukaBehaviorActionServer {
private:
    ros::NodeHandle nh;
    //RobotBehaviorInterface robotBehaviorInterface;
    actionlib::SimpleActionServer<kuka_move_as::RobotBehaviorAction> robot_behavior_as;
    kuka_move_as::RobotBehaviorGoal goal_;
    kuka_move_as::RobotBehaviorFeedback feedback_; 
    kuka_move_as::RobotBehaviorResult result_;
    bool isPreempt;
    bool goalComplete_;
    //RobotState robotState;
    unordered_map<int8_t, string> placeFinder;
    ros::Publisher joint_trajectory_publisher_;
    control_msgs::FollowJointTrajectoryGoal traj_goal_;
    trajectory_msgs::JointTrajectory traj_;
    trajectory_msgs::JointTrajectory jspace_pose_to_traj(Eigen::VectorXd joints, double dtime=2.0);
    TransitionTrajectories transitionTrajectories_;
    //callback fnc for trajectory action server interface to Kuka robots
    void trajCtlrCb(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result);

    //void send_traj_goal(trajectory_msgs::JointTrajectory des_trajectory);
    //void trajDoneCb_(const actionlib::SimpleClientGoalState& state,
    //    const control_msgs::FollowJointTrajectoryResultConstPtr& result);

    control_msgs::FollowJointTrajectoryGoal robot_goal_;
    trajectory_msgs::JointTrajectory des_trajectory_;
    
    //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    //        robot_motion_action_client("/ariac/arm/follow_joint_trajectory", true);
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> traj_ctl_ac_;

    
    //here are the action functions: robot moves
    /*
    void move_to_jspace_pose(Eigen::VectorXd q_vec, double dtime=2.0); //case robot_move_as::RobotMoveGoal::TO_PREDEFINED_POSE:
    unsigned short int flip_part_fnc(const robot_move_as::RobotMoveGoalConstPtr& goal); 
    unsigned short int grasp_fnc(double timeout=2.0);  //default timeout; rtns error code
    unsigned short int release_fnc(double timeout=2.0); //default timeout for release
    unsigned short int pick_part_fnc(const robot_move_as::RobotMoveGoalConstPtr& goal); //rtns err code; used within other fncs
    unsigned short int place_part_fnc_no_release(inventory_msgs::Part part);
    unsigned short int move_part(const robot_move_as::RobotMoveGoalConstPtr &goal,double timeout=0);   
    unsigned short int is_pickable(const robot_move_as::RobotMoveGoalConstPtr &goal);
    unsigned short int is_placeable(inventory_msgs::Part part);
    */
    int current_pose_code_;

    /*
    Eigen::VectorXd q_des_7dof_,q_cruise_pose_,bin_cruise_jspace_pose_,bin_hover_jspace_pose_;
    //Eigen::VectorXd agv_hover_pose_,agv_cruise_pose_;
    Eigen::VectorXd source_hover_pose_,source_cruise_pose_; 
    Eigen::VectorXd destination_hover_pose_,destination_cruise_pose_;
    double source_track_displacement_,destination_track_displacement_;
    
    Eigen::VectorXd q_Q1_rvrs_discard_,q_Q1_rvrs_hover_,q_Q1_rvrs_hover_flip_,q_Q1_rvrs_cruise_;
    Eigen::VectorXd q_Q1_fwd_discard_,q_Q1_fwd_hover_,q_Q1_fwd_hover_flip_,q_Q1_fwd_cruise_;
    Eigen::VectorXd q_Q1_arm_vertical_;
    Eigen::VectorXd q_Q1_dropoff_near_left_,q_Q1_dropoff_far_left_,q_Q1_dropoff_near_right_,q_Q1_dropoff_far_right_;

    
    //get rid of some of  these!
    
    
    Eigen::VectorXd box_hover_pose_,box_cruise_pose_;   
    
    Eigen::VectorXd q_box_Q2_hover_pose_,q_box_Q2_cruise_pose_;   
    Eigen::VectorXd q_Q1_discard_pose_,q_Q2_discard_pose_;
    Eigen::VectorXd q_Q1_cruise_pose_,q_Q1_hover_pose_;
    Eigen::VectorXd q_bin_cruise_pose_,q_destination_cruise_pose_;
    Eigen::VectorXd q_manip_nom_;
    
    Eigen::VectorXd q_init_pose_,q_hover_pose_;
     *     */
    Eigen::VectorXd pickup_jspace_pose_,dropoff_jspace_pose_;
    Eigen::VectorXd approach_pickup_jspace_pose_,approach_dropoff_jspace_pose_;
    
    
    //Eigen::VectorXd q_agv1_hover_pose_,q_agv1_cruise_pose_;  
    //Eigen::VectorXd q_agv2_hover_pose_,q_agv2_cruise_pose_;    
    /*
    Eigen::VectorXd q_conveyor_hover_pose_,q_conveyor_cruise_pose_;    
    Eigen::VectorXd q_bin8_cruise_pose_,q_bin8_hover_pose_,q_bin8_retract_pose_;    
    Eigen::VectorXd q_bin7_cruise_pose_,q_bin7_hover_pose_,q_bin7_retract_pose_;  
    Eigen::VectorXd q_bin6_cruise_pose_,q_bin6_hover_pose_,q_bin6_retract_pose_;  
    Eigen::VectorXd q_bin5_cruise_pose_,q_bin5_hover_pose_,q_bin5_retract_pose_;  
    Eigen::VectorXd q_bin4_cruise_pose_,q_bin4_hover_pose_,q_bin4_retract_pose_;  
    Eigen::VectorXd q_bin3_cruise_pose_,q_bin3_hover_pose_,q_bin3_retract_pose_;  
    Eigen::VectorXd q_bin2_cruise_pose_,q_bin2_hover_pose_,q_bin2_retract_pose_;  
    Eigen::VectorXd q_bin1_cruise_pose_,q_bin1_hover_pose_,q_bin1_retract_pose_;
    Eigen::VectorXd q_bin_pulley_flip_;
     * */
    Eigen::Affine3d grasp_transform_;
    //Eigen::VectorXd j1;
    
    Eigen::Affine3d affine_vacuum_pickup_pose_wrt_base_link_;
    Eigen::Affine3d affine_vacuum_dropoff_pose_wrt_base_link_;

    //void set_key_poses();
    //fncs to get key joint-space poses:
    //each bin gets a corresponding rail pose; return "true" if valid bin code
   // bool rail_prepose(int8_t location, double &q_rail);
    //each bin has a corresponding "hover" pose; set q_vec and return true if valid bin code
    //bool bin_hover_jspace_pose(int8_t bin, Eigen::VectorXd &q_vec);
    //cruise pose depends on bin code and whether to point towards agv1 or agv2
    // provide bin code and agv code; get back q_vec to prepare for cruise to agv
    //bool bin_cruise_jspace_pose(int8_t bin, int8_t agv, Eigen::VectorXd &q_vec);
    //bool bin_cruise_jspace_pose(int8_t location, Eigen::VectorXd &q_vec);
    

    /*
    bool hover_jspace_pose_w_code(int8_t bin, unsigned short int box_placement_location_code, Eigen::VectorXd &q_vec);
    bool hover_jspace_pose(int8_t bin, Eigen::VectorXd &q_vec); //{ hover_jspace_pose(bin,(unsigned short int) 0,&q_vec)};  //default, no box location  code    
    //bool RobotMoveActionServer::hover_jspace_pose(int8_t bin, int8_t box_placement_location_code, Eigen::VectorXd &qvec)
    bool cruise_jspace_pose_w_code(int8_t bin, unsigned short int box_placement_location_code, Eigen::VectorXd &q_vec);
    bool cruise_jspace_pose(int8_t bin,  Eigen::VectorXd &q_vec); // { return cruise_jspace_pose(bin,robot_move_as::RobotMoveGoal::Q1_DROPOFF_UNKNOWN,&q_vec);}; //default, no box location code
    bool set_q_manip_nom_from_destination_part(Part part);
    //trivial func to compute affine3 for robot_base w/rt world;  only depends on rail displacement
    Eigen::Affine3d  affine_base_link(double q_rail);

    double get_pickup_offset(Part part); //fnc to return offset values for gripper: part top relative to part frame
    double get_dropoff_offset(Part part);
    double get_surface_height(Part part);
    
    //"Part" should include part pose w/rt world, so can determine if part is right-side up or up-side down
    bool get_grasp_transform(Part part,Eigen::Affine3d &grasp_transform);



    bool eval_up_down(geometry_msgs::PoseStamped part_pose_wrt_world);
    //given rail displacement, and given Part description (including name and pose info) compute where the gripper should be, as
    //an Affine3 w/rt base_link frame
    Eigen::Affine3d affine_vacuum_pickup_pose_wrt_base_link(Part part, double q_rail);
    //similarly, compute gripper pose for dropoff, accounting for part height
    Eigen::Affine3d affine_vacuum_dropoff_pose_wrt_base_link(Part part, double q_rail);
    //do IK to place gripper at specified affine3; choose solution that is closest to provided jspace pose
    bool get_pickup_dropoff_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd approx_jspace_pose,Eigen::VectorXd &q_vec_soln);

    bool compute_approach_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd approx_jspace_pose,double approach_dist,Eigen::VectorXd &q_vec_soln);


    void grab();
    void release();  
    //RobotState calcRobotState();
    osrf_gear::VacuumGripperState getGripperState();
    bool attached_;
    bool isGripperAttached();
    bool waitForGripperAttach(double timeout);
  
    ros::ServiceClient gripper_client;
    osrf_gear::VacuumGripperState currentGripperState_;
    osrf_gear::VacuumGripperControl attach_;
    osrf_gear::VacuumGripperControl detach_;
    
    tf::TransformListener* tfListener_;
    tf::StampedTransform tfCameraWrtWorld_,tfTray1WrtWorld_,tfTray2WrtWorld_;
    XformUtils xformUtils_;
    UR10FwdSolver fwd_solver_;
    UR10IkSolver ik_solver_;
    Eigen::Affine3d agv1_tray_frame_wrt_world_,agv2_tray_frame_wrt_world_;
    double approach_dist_;
    inventory_msgs::Part part_of_interest_;
     * */
public:
    KukaBehaviorActionServer(ros::NodeHandle nodeHandle, string topic);
    void executeCB(const kuka_move_as::RobotBehaviorGoalConstPtr &goal);
    void preemptCB();
    //bool get_pose_from_code(unsigned short int POSE_CODE, Eigen::VectorXd &q_vec);

};


#endif //ROBOT_MOVE_AS_ROBOT_MOVE_AS_H

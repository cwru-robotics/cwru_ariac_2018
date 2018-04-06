//
// Created by shipei on 2/17/17.
// modified wsn 3/5/18
//

#ifndef KUKA_BEHAVIOR_AS_H
#define KUKA_BEHAVIOR_AS_H

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
#include <kuka_move_as/GripperInterface.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tf/transform_listener.h>
#include <xform_utils/xform_utils.h>
#include <kuka_fk_ik/kuka_kin.h>


#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>
#include <inventory_msgs/Part.h>


using namespace std;
using namespace Eigen;
using namespace inventory_msgs;

const bool UP = true;
const bool DOWN = false;

const double MAX_BEHAVIOR_SERVER_WAIT_TIME = 30.0; //to prevent deadlocks

const double BOX_SURFACE_HT_WRT_WORLD = 0.585; // from Gazebo
const double APPROACH_OFFSET_DIST = 0.02;
const double DEEP_GRASP_MOVE_DIST = -0.06;

const double MIN_BIN_GRASP_DY = -0.140; //-0.153;
const double MAX_BIN_GRASP_DY = 0.180; //0.190;
const double MAX_BIN_X_VAL = -0.6;
const double MIN_BIN_X_VAL = -0.89;
const double MID_BIN_X_VAL = -0.745;
const double BIN_FAR_THRESHOLD = -0.83;
const double Y_BASE_WRT_WORLD_AT_D8_HOME = 1.01;
const double X_BASE_WRT_WORLD = -0.050;


int ans; //poor-man's debug response


//map inventory_msgs location codes to corresponding robot pose codes:
std::map<unsigned short int, int> location_to_pose_code_map ={
    {inventory_msgs::Part::BIN1, BIN1_HOVER_NEAR_CODE},
    {inventory_msgs::Part::BIN2, BIN2_HOVER_NEAR_CODE},
    {inventory_msgs::Part::BIN3, BIN3_HOVER_NEAR_CODE},
    {inventory_msgs::Part::BIN4, BIN4_HOVER_NEAR_CODE},
    {inventory_msgs::Part::BIN5, BIN5_HOVER_NEAR_CODE},
    {inventory_msgs::Part::QUALITY_SENSOR_1, Q1_HOVER_CODE},
    {inventory_msgs::Part::QUALITY_SENSOR_2, Q2_HOVER_CODE},
    {inventory_msgs::Part::DISCARD_Q1, Q1_DISCARD_CODE},
    {inventory_msgs::Part::DISCARD_Q2, Q2_DISCARD_CODE}
};

std::map<unsigned short int, int> location_to_cruise_code_map={
    {inventory_msgs::Part::BIN1, BIN1_CRUISE_CODE},
    {inventory_msgs::Part::BIN2, BIN2_CRUISE_CODE},
    {inventory_msgs::Part::BIN3, BIN3_CRUISE_CODE},
    {inventory_msgs::Part::BIN4, BIN4_CRUISE_CODE},
    {inventory_msgs::Part::BIN5, BIN5_CRUISE_CODE},
    {inventory_msgs::Part::QUALITY_SENSOR_1, Q1_CRUISE_CODE},
    {inventory_msgs::Part::QUALITY_SENSOR_2, Q2_CRUISE_CODE},
    {inventory_msgs::Part::DISCARD_Q1, Q1_DISCARD_CODE},
    {inventory_msgs::Part::DISCARD_Q2, Q2_DISCARD_CODE}
};

/*
 const int APPROACH_DEPART_CODE = 1;
const int GRASP_PLACE_CODE = 2;
const int GRASP_DEEPER_CODE = 3;

const int HOME_POSE_CODE = 4;
const int CRUISE_FLIP_MID_CODE = 5; //kuka_move_as::RobotBehaviorGoal::CRUISE_FLIP_MID_CODE;
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

 */
std::map<int, string> map_pose_code_to_name = {
    {APPROACH_DEPART_CODE, "APPROACH_DEPART_CODE"},
    {GRASP_PLACE_CODE, "GRASP_PLACE_CODE"},
    {GRASP_DEEPER_CODE, "GRASP_DEEPER_CODE"},
    {HOME_POSE_CODE, "HOME_POSE_CODE"},
    {INIT_POSE_CODE, "INIT_POSE_CODE"},
    {NOM_BIN_CRUISE, "NOM_BIN_CRUISE"},
    
    {BIN1_CRUISE_CODE, "BIN1_CRUISE_CODE"},
    {BIN1_HOVER_NEAR_CODE, "BIN1_HOVER_NEAR_CODE"},
    {BIN1_HOVER_FAR_CODE, "BIN1_HOVER_FAR_CODE"},
    
    {BIN2_CRUISE_CODE, "BIN2_CRUISE_CODE"},
    {BIN2_HOVER_NEAR_CODE, "BIN2_HOVER_NEAR_CODE"},
    {BIN2_HOVER_FAR_CODE, "BIN2_HOVER_FAR_CODE"},
    
    {BIN3_CRUISE_CODE, "BIN3_CRUISE_CODE"},
    {BIN3_HOVER_NEAR_CODE, "BIN3_HOVER_NEAR_CODE"},
    {BIN3_HOVER_FAR_CODE, "BIN3_HOVER_FAR_CODE"},    
    
    {BIN4_CRUISE_CODE, "BIN4_CRUISE_CODE"},
    {BIN4_HOVER_NEAR_CODE, "BIN4_HOVER_NEAR_CODE"},
    {BIN4_HOVER_FAR_CODE, "BIN4_HOVER_FAR_CODE"},    
    
    {BIN5_CRUISE_CODE, "BIN5_CRUISE_CODE"},
    {BIN5_HOVER_NEAR_CODE, "BIN5_HOVER_NEAR_CODE"},
    {BIN5_HOVER_FAR_CODE, "BIN5_HOVER_FAR_CODE"},      

    {Q1_HOVER_CODE, "Q1_HOVER_CODE"},
    {Q1_CRUISE_CODE, "Q1_CRUISE_CODE"},
    {Q1_DISCARD_CODE, "Q1_DISCARD_CODE"},  
    
    {Q2_HOVER_CODE, "Q2_HOVER_CODE"},
    {Q2_CRUISE_CODE, "Q2_CRUISE_CODE"},
    {Q2_DISCARD_CODE, "Q2_DISCARD_CODE"}     
};

std::map<short unsigned int, string> error_code_name_map = {
    {kuka_move_as::RobotBehaviorResult::NO_ERROR, "NO_ERROR"},
    {kuka_move_as::RobotBehaviorResult::WRONG_PARAMETER, "WRONG_PARAMETER"},
    {kuka_move_as::RobotBehaviorResult::TIMEOUT, "TIMEOUT"},
    {kuka_move_as::RobotBehaviorResult::GRIPPER_FAULT, "GRIPPER_FAULT"},
    {kuka_move_as::RobotBehaviorResult::PART_DROPPED, "PART_DROPPED"},
    {kuka_move_as::RobotBehaviorResult::PRECOMPUTED_TRAJ_ERR, "PRECOMPUTED_TRAJ_ERR"},
    {kuka_move_as::RobotBehaviorResult::CANCELLED, "CANCELLED"}    
};
/*
 int8 QUALITY_SENSOR_1=51
int8 QUALITY_SENSOR_2=52

int8 BIN1=1
int8 BIN2=2
int8 BIN3=3
int8 BIN4=4
int8 BIN5=5
int8 BIN6=6
int8 BIN7=7
int8 BIN8=8
int8 BIN9=9
int8 BIN10=10
int8 BIN11=11
int8 BIN12=12

int8 DISCARD=21
 */


class KukaBehaviorActionServer {
private:
    ros::NodeHandle nh;
    //RobotBehaviorInterface robotBehaviorInterface;
    actionlib::SimpleActionServer<kuka_move_as::RobotBehaviorAction> robot_behavior_as;
    kuka_move_as::RobotBehaviorGoal goal_;
    kuka_move_as::RobotBehaviorFeedback feedback_;
    kuka_move_as::RobotBehaviorResult result_;
    bool isPreempt_;
    //bool goalComplete_;
    //RobotState robotState;
    unordered_map<int8_t, string> placeFinder_;
    ros::Publisher joint_trajectory_publisher_;
    control_msgs::FollowJointTrajectoryGoal traj_goal_;
    trajectory_msgs::JointTrajectory traj_;
    trajectory_msgs::JointTrajectory jspace_pose_to_traj(Eigen::VectorXd joints, double dtime = 2.0);
    bool move_to_jspace_pose(Eigen::VectorXd desired_jspace_pose, double arrival_time);

    TransitionTrajectories transitionTrajectories_;
    GripperInterface gripperInterface_;
    
    //callback fnc for trajectory action server interface to Kuka robots
    void trajDoneCb_(const actionlib::SimpleClientGoalState& state,
            const control_msgs::FollowJointTrajectoryResultConstPtr& result);
    bool send_traj_goal(trajectory_msgs::JointTrajectory des_trajectory);
    bool send_traj_goal(trajectory_msgs::JointTrajectory des_trajectory, int destination_code);
    bool traj_goal_complete_;
    bool is_attached_;
    //    void armDoneCb_(const actionlib::SimpleClientGoalState& state,
    //    const control_msgs::FollowJointTrajectoryResultConstPtr& result);

    //void send_traj_goal(trajectory_msgs::JointTrajectory des_trajectory);
    //void trajDoneCb_(const actionlib::SimpleClientGoalState& state,
    //    const control_msgs::FollowJointTrajectoryResultConstPtr& result);

    control_msgs::FollowJointTrajectoryGoal robot_goal_;
    trajectory_msgs::JointTrajectory des_trajectory_;

    //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    //        robot_motion_action_client("/ariac/arm/follow_joint_trajectory", true);
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> traj_ctl_ac_;
    int current_pose_code_;
    unsigned short int errorCode_;

    bool move_posecode1_to_posecode2(int posecode_start, int posecode_goal);

    bool report_success_or_abort(); //typical wrap-up for a behavior
    //here are the action functions: robot moves
    //unsigned short int pick_part_fnc(const kuka_move_as::RobotBehaviorGoalConstPtr &goal);
    unsigned short int pick_part_from_bin(const kuka_move_as::RobotBehaviorGoalConstPtr &goal);
    //unsigned short int discard_grasped_part();
    unsigned short int discard_grasped_part(inventory_msgs::Part part);

    unsigned short int place_part_in_box_no_release(inventory_msgs::Part part); 
    
    bool move_to_jspace_pose(const int pose_code, double arrival_time);
    bool move_into_grasp(double arrival_time); //ASSUMES deep-grasp pose
    bool move_into_grasp(Eigen::VectorXd pickup_jspace_pose, double arrival_time); //provide target pose

    inventory_msgs::Part part_of_interest_;
    inventory_msgs::Part grasped_part_;

    Eigen::VectorXd pickup_jspace_pose_, dropoff_jspace_pose_;
    Eigen::VectorXd approach_pickup_jspace_pose_, approach_dropoff_jspace_pose_;
    Eigen::VectorXd pickup_deeper_jspace_pose_, pickup_hover_pose_, dropoff_hover_pose_, current_hover_pose_;
    Eigen::VectorXd desired_approach_depart_pose_, desired_grasp_dropoff_pose_;
    Eigen::VectorXd computed_jspace_approach_, computed_bin_escape_jspace_pose_;
    Eigen::VectorXd nom_bin_cruise_pose_,q1_cruise_pose_,computed_bin_cruise_jspace_pose_;
    Eigen::VectorXd  q1_hover_pose_,q2_hover_pose_;
    double approach_dist_;
    double deep_grasp_dist_;
    
    Eigen::Affine3d grasp_transform_;
    Eigen::Affine3d affine_vacuum_pickup_pose_wrt_base_link_;
    Eigen::Affine3d affine_vacuum_dropoff_pose_wrt_base_link_;
    
    //trivial func to compute affine3 for robot_base w/rt world;  only depends on rail displacement
    Eigen::Affine3d  affine_base_link(double q_rail);

    double get_pickup_offset(Part part); //fnc to return offset values for gripper: part top relative to part frame
    double get_dropoff_offset(Part part);
    //double get_surface_height(Part part);
    
    //"Part" should include part pose w/rt world, so can determine if part is right-side up or up-side down
    bool get_grasp_transform(Part part,Eigen::Affine3d &grasp_transform);
    bool eval_up_down(geometry_msgs::PoseStamped part_pose_wrt_world);
    //given rail displacement, and given Part description (including name and pose info) compute where the gripper should be, as
    //an Affine3 w/rt base_link frame
    Eigen::Affine3d affine_vacuum_pickup_pose_wrt_base_link(Part part, double q_rail);
    //similarly, compute gripper pose for dropoff, accounting for part height
    Eigen::Affine3d affine_vacuum_dropoff_pose_wrt_base_link(Part part, double q_rail);


    //void grab();
    //void release();  
    //unsigned short int grasp_fnc(double timeout=2.0);  //default timeout; rtns error code
    //unsigned short int release_fnc(double timeout=2.0); //default timeout for release    

    //osrf_gear::VacuumGripperState getGripperState();
    //bool attached_;
    //bool isGripperAttached();
    //bool waitForGripperAttach(double timeout);
  
    //ros::ServiceClient gripper_client;
    //osrf_gear::VacuumGripperState currentGripperState_;
    //osrf_gear::VacuumGripperControl attach_;
    //osrf_gear::VacuumGripperControl detach_;
    
    tf::TransformListener* tfListener_;
    //tf::StampedTransform tfCameraWrtWorld_;
    XformUtils xformUtils_;
    KukaFwdSolver fwd_solver_;
    KukaIkSolver ik_solver_;
    
    void copy_array_to_qvec(const double q_array[],Eigen::VectorXd &qvec);
    Eigen::VectorXd source_hover_pose_, destination_hover_pose_;
    
    bool hover_jspace_pose_w_code(int8_t bin, unsigned short int box_placement_location_code, Eigen::VectorXd &qvec);
    bool hover_jspace_pose(int8_t bin, Eigen::VectorXd &q_vec);
    bool rail_prepose(int8_t location, double &q_rail);
    bool compute_bin_hover_from_xy(double x_part,double y_part, Eigen::VectorXd &qvec);
    unsigned short int compute_bin_pickup_key_poses(inventory_msgs::Part part);    
    unsigned short int compute_box_dropoff_key_poses(inventory_msgs::Part part);
    //do IK to place gripper at specified affine3; choose solution that is closest to provided jspace pose
    bool compute_pickup_dropoff_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd approx_jspace_pose,Eigen::VectorXd &q_vec_soln);
    bool compute_approach_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd approx_jspace_pose,double approach_dist,Eigen::VectorXd &q_vec_soln);

    
    bool bin_center_y_coord(int8_t location, double &bin_y_val);
    //bool bin_y_is_reachable(int8_t bin,double &part_y);
    bool bin_xy_is_reachable(int8_t bin,double &part_x, double &part_y);


    //inventory_msgs::Part part_of_interest_;    
    
    //bool KukaBehaviorActionServer::move_to_jspace_pose(const int pose_code, double arrival_time) {


    /*
    void move_to_jspace_pose(Eigen::VectorXd q_vec, double dtime=2.0); //case robot_move_as::RobotMoveGoal::TO_PREDEFINED_POSE:
    unsigned short int flip_part_fnc(const robot_move_as::RobotMoveGoalConstPtr& goal); 

    unsigned short int pick_part_fnc(const robot_move_as::RobotMoveGoalConstPtr& goal); //rtns err code; used within other fncs
    unsigned short int place_part_fnc_no_release(inventory_msgs::Part part);
    unsigned short int move_part(const robot_move_as::RobotMoveGoalConstPtr &goal,double timeout=0);   
    unsigned short int is_pickable(const robot_move_as::RobotMoveGoalConstPtr &goal);
    unsigned short int is_placeable(inventory_msgs::Part part);
     */


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

     * */
public:
    KukaBehaviorActionServer(ros::NodeHandle nodeHandle, string topic);
    void executeCB(const kuka_move_as::RobotBehaviorGoalConstPtr &goal);
    void preemptCB();
    //bool get_pose_from_code(unsigned short int POSE_CODE, Eigen::VectorXd &q_vec);

};


#endif //ROBOT_MOVE_AS_ROBOT_MOVE_AS_H

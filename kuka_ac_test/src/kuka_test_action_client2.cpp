// kuka_test_action_client2:  does:
//INIT-> CRUISE_FLIP_MID -> BIN2_CRUISE_POSE  -> BIN2_HOVER_LEFT_NEAR -> BIN2_HOVER_LEFT_FAR ->
// BIN2_HOVER_LEFT_NEAR -> BIN2_CRUISE_POSE -> CRUISE_FLIP_MID -> Q1_CRUISE -> Q1_HOVER -> Q1_CRUISE


#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

int g_done_count = 0;

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    g_done_count++;
}

int main(int argc, char** argv) {
        ros::init(argc, argv, "demo_trajectory_client_node"); // name this node 
        ros::NodeHandle nh; //standard ros node handle        

        control_msgs::FollowJointTrajectoryGoal robot_goal;
        //instantiate a goal message compatible with robot action server
        trajectory_msgs::JointTrajectory des_trajectory;
    //instantiate an action client of the robot-arm motion  action server:
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
            robot_motion_action_client("/ariac/arm/follow_joint_trajectory", true);
    
        Eigen::VectorXd q_CRUISE_FLIP_MID,q_BIN2_CRUISE_POSE, q_BIN2_HOVER_LEFT_NEAR;
        Eigen::VectorXd q_Q1_CRUISE,q_Q1_HOVER,q_BIN2_HOVER_LEFT_FAR;
        q_CRUISE_FLIP_MID.resize(8); 
        q_BIN2_CRUISE_POSE.resize(8);
        q_BIN2_HOVER_LEFT_NEAR.resize(8);
        q_Q1_CRUISE.resize(8);
        q_Q1_HOVER.resize(8);
        q_BIN2_HOVER_LEFT_FAR.resize(8);
        q_Q1_HOVER<< 0.0, 1.1, 0.0, -1.6, 0, 0.45, 0, -0.35;
        q_Q1_CRUISE<< 1.57, 1.1, 0.0, -1.6, 0, 0.45, 0, -0.35;
        q_CRUISE_FLIP_MID << 1.57, 1.1, 0.0, -1.6, 0, 0.45, 0, -0.35;   
        q_BIN2_CRUISE_POSE << 1.57, -1.35, 0.0, 0.5, 0, -1.2, 0, 0;
        q_BIN2_HOVER_LEFT_NEAR<< 0.5, -1.35, 0.0, 0.5, 0, -1.2, 0, -0.4;
        q_BIN2_HOVER_LEFT_FAR<< 0.1, -1.35, 0.0, 0.3, 0, -1.2, 0, -0.7;

    // attempt to connect to the server:
    ROS_INFO("waiting for arm server: ");
    bool server_exists = robot_motion_action_client.waitForServer(ros::Duration(1.0));
    while (!server_exists) {
        ROS_WARN("waiting on arm server...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        server_exists = robot_motion_action_client.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to arm action server"); // if here, then we connected to the server;  

     //populate a goal message
        Eigen::VectorXd q_goal;
        q_goal.resize(8);
        //q_goal<<0,0,0,0,0,0,0,0.5;
        q_goal<<0.0, 1.1, 0.0, -1.4, 0, 0.6, 1, 0.0; //box hover pose:

        trajectory_msgs::JointTrajectoryPoint trajectory_point1;
        trajectory_point1.positions.clear();
        trajectory_point1.positions.resize(8);
        
        des_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
        des_trajectory.joint_names.clear();
    //[iiwa_joint_1, iiwa_joint_2, iiwa_joint_3, iiwa_joint_4, iiwa_joint_5, iiwa_joint_6,
    //iiwa_joint_7, linear_arm_actuator_joint
    des_trajectory.joint_names.push_back("iiwa_joint_1");
    des_trajectory.joint_names.push_back("iiwa_joint_2");
    des_trajectory.joint_names.push_back("iiwa_joint_3");
    des_trajectory.joint_names.push_back("iiwa_joint_4");
    des_trajectory.joint_names.push_back("iiwa_joint_5");
    des_trajectory.joint_names.push_back("iiwa_joint_6");
    des_trajectory.joint_names.push_back("iiwa_joint_7");
    des_trajectory.joint_names.push_back("linear_arm_actuator_joint");


    //q_CRUISE_FLIP_MID:    
    for (int i = 0; i < 8; i++) { //copy over the joint-command values
        trajectory_point1.positions[i] = q_CRUISE_FLIP_MID[i];
    }


    trajectory_point1.time_from_start = ros::Duration(2.0);
    des_trajectory.points.push_back(trajectory_point1);
    
    //q_BIN2_CRUISE_POSE
    for (int i = 0; i < 8; i++) { //copy over the joint-command values
        trajectory_point1.positions[i] = q_BIN2_CRUISE_POSE[i];
    }

    //set arrival time:
    trajectory_point1.time_from_start = ros::Duration(4.0);
    des_trajectory.points.push_back(trajectory_point1);
    
    //q_BIN2_HOVER_LEFT_NEAR
    for (int i = 0; i < 8; i++) { //copy over the joint-command values
        trajectory_point1.positions[i] = q_BIN2_HOVER_LEFT_NEAR[i];
    }

    //set arrival time:
    trajectory_point1.time_from_start = ros::Duration(6.0);
    des_trajectory.points.push_back(trajectory_point1);    
    
    //q_BIN2_HOVER_LEFT_FAR
    for (int i = 0; i < 8; i++) { //copy over the joint-command values
        trajectory_point1.positions[i] = q_BIN2_HOVER_LEFT_FAR[i];
    }

    //set arrival time:
    trajectory_point1.time_from_start = ros::Duration(8.0);
    des_trajectory.points.push_back(trajectory_point1);     
    
    //q_BIN2_HOVER_LEFT_NEAR
    for (int i = 0; i < 8; i++) { //copy over the joint-command values
        trajectory_point1.positions[i] = q_BIN2_HOVER_LEFT_NEAR[i];
    }

    //set arrival time:
    trajectory_point1.time_from_start = ros::Duration(10.0);
    des_trajectory.points.push_back(trajectory_point1);   

        //q_BIN2_CRUISE_POSE
    for (int i = 0; i < 8; i++) { //copy over the joint-command values
        trajectory_point1.positions[i] = q_BIN2_CRUISE_POSE[i];
    }

    //set arrival time:
    trajectory_point1.time_from_start = ros::Duration(12.0);
    des_trajectory.points.push_back(trajectory_point1);
    
    //q_CRUISE_FLIP_MID:    
    for (int i = 0; i < 8; i++) { //copy over the joint-command values
        trajectory_point1.positions[i] = q_CRUISE_FLIP_MID[i];
    }
    trajectory_point1.time_from_start = ros::Duration(14.0);
    des_trajectory.points.push_back(trajectory_point1);
      
    //q_Q1_CRUISE    
    for (int i = 0; i < 8; i++) { //copy over the joint-command values
        trajectory_point1.positions[i] = q_Q1_CRUISE[i];
    }
    trajectory_point1.time_from_start = ros::Duration(16.0);
    des_trajectory.points.push_back(trajectory_point1);  
    
    //q_Q1_HOVER    
    for (int i = 0; i < 8; i++) { //copy over the joint-command values
        trajectory_point1.positions[i] = q_Q1_HOVER[i];
    }
    trajectory_point1.time_from_start = ros::Duration(18.0);
    des_trajectory.points.push_back(trajectory_point1);  

    //q_Q1_CRUISE    
    for (int i = 0; i < 8; i++) { //copy over the joint-command values
        trajectory_point1.positions[i] = q_Q1_CRUISE[i];
    }
    trajectory_point1.time_from_start = ros::Duration(20.0);
    des_trajectory.points.push_back(trajectory_point1);      

    //put traj in goal message
    robot_goal.trajectory = des_trajectory;


    ROS_INFO("sending goal to arm: ");
    robot_motion_action_client.sendGoal(robot_goal, &doneCb);

    while (g_done_count < 1) {
        ROS_INFO("waiting for trajectory to finish...");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("done w/ test");

    return 0;
}

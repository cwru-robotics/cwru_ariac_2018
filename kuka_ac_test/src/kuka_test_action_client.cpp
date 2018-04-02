// example action client for Kuka robot action  server 
// wsn,  3/2018

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

int g_done_count = 0; //flag to indicate action server has returned a result

void doneCb(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    g_done_count++;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_kuka_client_node");
    ros::NodeHandle nh;

    control_msgs::FollowJointTrajectoryGoal robot_goal;
    //instantiate a goal message compatible with robot action server
    trajectory_msgs::JointTrajectory des_trajectory;
    //instantiate an action client of the robot-arm motion action server:
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
            robot_motion_action_client("/ariac/arm/follow_joint_trajectory", true);

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

    //populate a (trivial) trajectory message:
    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    trajectory_point.positions.resize(8);
    for (int i = 0; i < 8; i++) trajectory_point.positions[i] = 0.0; //init to zero
    trajectory_point.positions[0] = 1.57; //rotate joint-1 by about 90 deg

    des_trajectory.points.clear(); //not really necessary; just paranoid
    des_trajectory.joint_names.clear(); //ditto
    //set the joint names:
    des_trajectory.joint_names.push_back("iiwa_joint_1");
    des_trajectory.joint_names.push_back("iiwa_joint_2");
    des_trajectory.joint_names.push_back("iiwa_joint_3");
    des_trajectory.joint_names.push_back("iiwa_joint_4");
    des_trajectory.joint_names.push_back("iiwa_joint_5");
    des_trajectory.joint_names.push_back("iiwa_joint_6");
    des_trajectory.joint_names.push_back("iiwa_joint_7");
    des_trajectory.joint_names.push_back("linear_arm_actuator_joint");

    //set arrival time for a single point in the trajectory:
    trajectory_point.time_from_start = ros::Duration(2.0); //allow 2 sec for this move
    des_trajectory.points.push_back(trajectory_point);

    //put traj in goal message
    robot_goal.trajectory = des_trajectory;

    ROS_INFO("sending goal to arm: ");
    robot_motion_action_client.sendGoal(robot_goal, &doneCb);

    while (g_done_count < 1) {
        ROS_INFO("waiting to finish pre-pose..");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("done w/ test");
    ROS_INFO("sending command again");
    g_done_count=0;
        robot_motion_action_client.sendGoal(robot_goal, &doneCb);
    while (g_done_count < 1) {
        ROS_INFO("waiting to finish pre-pose..");
        ros::Duration(1.0).sleep();
    }

    return 0;
}

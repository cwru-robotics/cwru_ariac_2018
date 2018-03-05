//
// Created by shipei on 10/18/16.
//

#ifndef CWRU_ARIAC_ROBOTINTERFACE_H
#define CWRU_ARIAC_ROBOTINTERFACE_H

//#include <AriacBase.h>

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <chrono>
#include <mutex>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <osrf_gear/Order.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Proximity.h>
//#include <osrf_gear/AGVControl.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>

#include <inventory_msgs/Part.h>

class RobotInterface {
public:
    RobotInterface(ros::NodeHandle& nodeHandle);

    void sendJointsValue(vector<double> joints);
    vector<double> getJointsState();
    vector<string> getJointsNames();
    void grab();
    void release();
    osrf_gear::VacuumGripperState getGripperState();
    bool isGripperAttached();
    bool waitForGripperAttach(double timeout);

protected:
    ros::NodeHandle nh;

    ros::Publisher joint_trajectory_publisher;
    ros::Subscriber joint_state_subscriber;
    ros::ServiceClient gripper;
    ros::Subscriber gripperStateSubscriber;

    sensor_msgs::JointState current_joint_states;
    osrf_gear::VacuumGripperState currentGripperState;
    bool called;
    bool attached;
    osrf_gear::VacuumGripperControl attach;
    osrf_gear::VacuumGripperControl detach;
    double arrivalTime;

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_state_msg);
    void gripperStateCallback(const osrf_gear::VacuumGripperState::ConstPtr &msg);
};


#endif //CWRU_ARIAC_ROBOTINTERFACE_H

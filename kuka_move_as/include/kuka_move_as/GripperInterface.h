//
// wsn 4/2/18

#ifndef GRIPPER_INTERFACE_H
#define GRIPPER_INTERFACE_H

#include <ros/ros.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>
#include <robot_behavior_interface/RobotBehaviorAction.h>

using namespace std;

class GripperInterface {
public:
    GripperInterface(ros::NodeHandle& nodeHandle);

    void grab();
    void release();
    short unsigned  int release_fnc(double timeout);
    //short unsigned int grasp_fnc();
    osrf_gear::VacuumGripperState getGripperState();
    bool isGripperAttached();
    bool waitForGripperAttach(double timeout);

protected:
    ros::NodeHandle nh;
    ros::ServiceClient gripper_;
    ros::Subscriber gripperStateSubscriber;
    osrf_gear::VacuumGripperState currentGripperState_;
    bool attached_;
    osrf_gear::VacuumGripperControl attach_;
    osrf_gear::VacuumGripperControl detach_;
    void gripperStateCallback(const osrf_gear::VacuumGripperState::ConstPtr &msg);
    
};


#endif 

//
// modified wsn 3/2018
//

#include <kuka_move_as/GripperInterface.h>

GripperInterface::GripperInterface(ros::NodeHandle &nodeHandle): nh( nodeHandle ){

    gripper_ = nh.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");
    gripperStateSubscriber = nh.subscribe("/ariac/gripper/state", 10, &GripperInterface::gripperStateCallback, this);
    attached_ = false;

    if (!gripper_.exists()) {
        gripper_.waitForExistence();
    }
    attach_.request.enable = 1;
    detach_.request.enable = 0;
}


void GripperInterface::gripperStateCallback(const osrf_gear::VacuumGripperState::ConstPtr &msg) {
    currentGripperState_ = *msg;
    attached_ = msg->attached;
}

osrf_gear::VacuumGripperState GripperInterface::getGripperState() {
    ros::spinOnce();
    return currentGripperState_;
}

bool GripperInterface::isGripperAttached() {
    ros::spinOnce();
    return attached_;
}

bool GripperInterface::waitForGripperAttach(double timeout) {
    timeout = timeout <= 0? FLT_MAX:timeout;
    ros::spinOnce();
    double dt = 0.1;
    while((!attached_) && timeout > 0 && ros::ok()) {
        ROS_INFO("testing for gripper attachment");
        ros::Duration(dt).sleep();
        ros::spinOnce();
        grab();
        timeout -= dt;
    }
    return attached_;
}


void GripperInterface::grab() {
    //ROS_INFO("enable gripper");
    gripper_.call(attach_);
}

void GripperInterface::release() {
    //ROS_INFO("release gripper");
    gripper_.call(detach_);
}


short unsigned  int GripperInterface::release_fnc(double timeout) {
      release();//attempt  release
     attached_ = isGripperAttached();
     short unsigned int errorCode = robot_behavior_interface::RobotBehaviorResult::NO_ERROR; 
     double timer=0;
     double dt =0.1;
     while (!attached_ && (timer<timeout)) {
        ROS_WARN("trying to release part");
        timer+=dt;
        release();
        ros::spinOnce();
        attached_ = isGripperAttached();     
    }
     if (!attached_) {
        errorCode = robot_behavior_interface::RobotBehaviorResult::GRIPPER_FAULT; //debug--return error
        return errorCode;
    }   
    //errorCode = robot_behavior_interface::RobotBehaviorResult::NO_ERROR; //return success
    return errorCode;
}

//    short unsigned int grasp_fnc();

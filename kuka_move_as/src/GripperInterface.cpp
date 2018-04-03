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
    while((!attached_) && timeout > 0 && ros::ok()) {
        ROS_INFO("Retry grasp");
        //release();
        ros::Duration(0.2).sleep();
        ros::spinOnce();
        grab();
        ros::Duration(0.8).sleep();
        ros::spinOnce();
        timeout -= 1.0;
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

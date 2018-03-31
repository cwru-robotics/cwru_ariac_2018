//
//  wsn 3/2018
//

#include <kuka_move_as/RobotBehaviorInterface.h>
RobotBehaviorInterface::RobotBehaviorInterface(ros::NodeHandle* nodehandle) : nh_(*nodehandle),ac("robot_behavior_server", true) {
    ROS_INFO("constructor of RobotBehaviorInterface");
    errorCodeFinder.insert(pair<int8_t, string>(kuka_move_as::RobotBehaviorResult::NO_ERROR, "NO_ERROR"));
    errorCodeFinder.insert(pair<int8_t, string>(kuka_move_as::RobotBehaviorResult::CANCELLED, "CANCELLED"));
    errorCodeFinder.insert(pair<int8_t, string>(kuka_move_as::RobotBehaviorResult::WRONG_PARAMETER, "WRONG_PARAMETER"));
    errorCodeFinder.insert(pair<int8_t, string>(kuka_move_as::RobotBehaviorResult::TIMEOUT, "TIMEOUT"));
    errorCodeFinder.insert(pair<int8_t, string>(kuka_move_as::RobotBehaviorResult::UNREACHABLE, "UNREACHABLE"));
    errorCodeFinder.insert(pair<int8_t, string>(kuka_move_as::RobotBehaviorResult::GRIPPER_FAULT, "GRIPPER_FAULT"));
    errorCodeFinder.insert(pair<int8_t, string>(kuka_move_as::RobotBehaviorResult::COLLISION, "COLLISION"));
    errorCodeFinder.insert(pair<int8_t, string>(kuka_move_as::RobotBehaviorResult::PART_DROPPED, "PART_DROPPED"));  
    
    ROS_INFO("waiting for robot_behavior_server: ");
    bool server_exists = ac.waitForServer(ros::Duration(1.0));
    while (!server_exists) {
        ROS_WARN("waiting robot_behavior_server...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        server_exists = ac.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to robot_behavior_server"); // if here, then we connected to the server;   

    //gripper = nh.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");
    //gripperStateSubscriber = nh.subscribe("/ariac/gripper/state", 10, &RobotInterface::gripperStateCallback, this);
    /*
    called = false;
    attached = false;
    while(!called && ros::ok()) {
        //ROS_INFO("Waiting for joint feedback...");
        ros::spinOnce();
        ros::Duration(0.02).sleep();
    }
    if (!gripper.exists()) {
        gripper.waitForExistence();
    }
    attach.request.enable = 1;
    detach.request.enable = 0;
    arrivalTime = 0.5;
     * */
}

void RobotBehaviorInterface::sendGoal(kuka_move_as::RobotBehaviorGoal goal) {
    action_server_returned_ = false;
    goal_start_time_ = ros::Time::now();
    ac.sendGoal(goal, boost::bind(&RobotBehaviorInterface::doneCb, this, _1, _2), boost::bind(&RobotBehaviorInterface::activeCb, this), boost::bind(&RobotBehaviorInterface::feedbackCb, this, _1));
}

void RobotBehaviorInterface::doneCb(const actionlib::SimpleClientGoalState &state, const kuka_move_as::RobotBehaviorResultConstPtr &result) {
    action_server_returned_ = true;
    goal_success_ = result->success;
    errorCode = result->errorCode;
    //currentRobotState = result->robotState;
    ROS_INFO("Action finished  with result error code: %d", (int) errorCode);
//    ROS_INFO("Gripper position is: %f, %f, %f\n",
//             currentRobotState.gripperPose.pose.position.x, currentRobotState.gripperPose.pose.position.y,
//             currentRobotState.gripperPose.pose.position.z);
//    showJointState(currentRobotState.jointNames, currentRobotState.jointStates);
}

void RobotBehaviorInterface::feedbackCb(const kuka_move_as::RobotBehaviorFeedbackConstPtr &feedback) {
    //do nothing
    //currentRobotState = feedback->robotState;
}

void RobotBehaviorInterface::cancel() {
    ac.cancelGoal();
}

void RobotBehaviorInterface::activeCb() {
    // ROS_INFO("Goal sent");
}


bool RobotBehaviorInterface::pick(Part part, double timeout) {
    ROS_INFO("pick fnc called");
    kuka_move_as::RobotBehaviorGoal goal;
    goal.type = kuka_move_as::RobotBehaviorGoal::PICK;
    goal.timeout = timeout;
    goal.sourcePart = part;
    ROS_INFO("sending goal message");
    sendGoal(goal);
    while (!action_server_returned_) {
        ROS_INFO("waiting on action  server");
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }
    /*
    if (!async_mode) {
        bool finished_before_timeout;
        if (timeout == 0) {
            finished_before_timeout = ac.waitForResult();
        } else {
            finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
        }
        if (!finished_before_timeout) {
            errorCode = kuka_move_as::RobotBehaviorResult::TIMEOUT;
        }
        return finished_before_timeout && goal_success_;
    }*/
    return true;
}


/*
void RobotInterface::gripperStateCallback(const osrf_gear::VacuumGripperState::ConstPtr &msg) {
    currentGripperState = *msg;
    attached = msg->attached;
}

osrf_gear::VacuumGripperState RobotInterface::getGripperState() {
    ros::spinOnce();
    return currentGripperState;
}

bool RobotInterface::isGripperAttached() {
    ros::spinOnce();
    return attached;
}

bool RobotInterface::waitForGripperAttach(double timeout) {
    timeout = timeout <= 0? FLT_MAX:timeout;
    ros::spinOnce();
    while((!attached) && timeout > 0 && ros::ok()) {
        ROS_INFO("Retry grasp");
        release();
        ros::Duration(0.2).sleep();
        ros::spinOnce();
        grab();
        ros::Duration(0.8).sleep();
        ros::spinOnce();
        timeout -= 1.0;
    }
    return attached;
}




void RobotInterface::grab() {
    //ROS_INFO("enable gripper");
    gripper.call(attach);
}

void RobotInterface::release() {
    //ROS_INFO("release gripper");
    gripper.call(detach);
}

 * */
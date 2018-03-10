//
// Created by tianshipei on 12/4/16.
//edited wsn 2/18/17
//
#include<robot_move_as/RobotMove.h>
//#include "RobotMove.h"

//RobotMove::RobotMove(ros::NodeHandle &nodeHandle, string topic): nh(nodeHandle), ac(topic, true) {
//BinInventory::BinInventory(ros::NodeHandle* nodehandle) : nh_(*nodehandle)
RobotMove::RobotMove(ros::NodeHandle* nodehandle) : nh_(*nodehandle),ac("robot_move", true)
{

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer(); //will wait for infinite time
    time_tolerance = 1.0;
    async_mode = false;
    ROS_INFO("Robot Move Action Client is ready.");
    errorCodeFinder.insert(pair<int8_t, string>(robot_move_as::RobotMoveResult::NO_ERROR, "NO_ERROR"));
    errorCodeFinder.insert(pair<int8_t, string>(robot_move_as::RobotMoveResult::CANCELLED, "CANCELLED"));
    errorCodeFinder.insert(pair<int8_t, string>(robot_move_as::RobotMoveResult::WRONG_PARAMETER, "WRONG_PARAMETER"));
    errorCodeFinder.insert(pair<int8_t, string>(robot_move_as::RobotMoveResult::TIMEOUT, "TIMEOUT"));
    errorCodeFinder.insert(pair<int8_t, string>(robot_move_as::RobotMoveResult::UNREACHABLE, "UNREACHABLE"));
    errorCodeFinder.insert(pair<int8_t, string>(robot_move_as::RobotMoveResult::GRIPPER_FAULT, "GRIPPER_FAULT"));
    errorCodeFinder.insert(pair<int8_t, string>(robot_move_as::RobotMoveResult::COLLISION, "COLLISION"));
    errorCodeFinder.insert(pair<int8_t, string>(robot_move_as::RobotMoveResult::PART_DROPPED, "PART_DROPPED"));
}
bool RobotMove::toHome(double timeout) {
    robot_move_as::RobotMoveGoal goal;
    goal.type = robot_move_as::RobotMoveGoal::TO_HOME;
    goal.timeout = timeout;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout;
        if (timeout == 0) {
            finished_before_timeout = ac.waitForResult();
        } else {
            finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
        }
        if (!finished_before_timeout) {
            errorCode = robot_move_as::RobotMoveResult::TIMEOUT;
        }
        return finished_before_timeout && goal_success_;
    }
    return true;
}

bool RobotMove::toPredefinedPose(int8_t predefined_pose_code, double timeout) {
    ROS_INFO("requesting move to pose code %d", predefined_pose_code);
    robot_move_as::RobotMoveGoal goal;
    goal.type = robot_move_as::RobotMoveGoal::TO_PREDEFINED_POSE;
    goal.predfinedPoseCode = predefined_pose_code;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout;
        if (timeout == 0) {
            finished_before_timeout = ac.waitForResult();
        } else {
            finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
        }
        if (!finished_before_timeout) {
            errorCode = robot_move_as::RobotMoveResult::TIMEOUT;
        }
        return finished_before_timeout && goal_success_;
    }
    return true;
}

bool RobotMove::flipPart(Part part, double timeout) {
    robot_move_as::RobotMoveGoal goal;
    goal.type = robot_move_as::RobotMoveGoal::FLIP_PART;
    goal.timeout = 0; //timeout;
    goal.sourcePart = part;
    //goal.targetPart = destination;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout;
        if (timeout == 0) {
            finished_before_timeout = ac.waitForResult();
        } else {
            finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
        }
        if (!finished_before_timeout) {
            errorCode = robot_move_as::RobotMoveResult::TIMEOUT;
        }
        return finished_before_timeout && goal_success_;
    }
    return true;
}

bool RobotMove::pick(Part part, double timeout) {
    robot_move_as::RobotMoveGoal goal;
    goal.type = robot_move_as::RobotMoveGoal::PICK;
    goal.timeout = timeout;
    goal.sourcePart = part;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout;
        if (timeout == 0) {
            finished_before_timeout = ac.waitForResult();
        } else {
            finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
        }
        if (!finished_before_timeout) {
            errorCode = robot_move_as::RobotMoveResult::TIMEOUT;
        }
        return finished_before_timeout && goal_success_;
    }
    return true;
}

bool RobotMove::place(Part destination, double timeout) {
    robot_move_as::RobotMoveGoal goal;
    goal.type = robot_move_as::RobotMoveGoal::PLACE;
    goal.timeout = timeout;
    goal.targetPart = destination;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout;
        if (timeout == 0) {
            finished_before_timeout = ac.waitForResult();
        } else {
            finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
        }
        if (!finished_before_timeout) {
            errorCode = robot_move_as::RobotMoveResult::TIMEOUT;
        }
        return finished_before_timeout && goal_success_;
    }
    return true;
}

bool RobotMove::place_part_no_release(Part destination, double timeout) {
    robot_move_as::RobotMoveGoal goal;
    goal.type = robot_move_as::RobotMoveGoal::PLACE_PART_NO_RELEASE;
    goal.timeout = timeout;
    goal.targetPart = destination;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout;
        if (timeout == 0) {
            finished_before_timeout = ac.waitForResult();
        } else {
            finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
        }
        if (!finished_before_timeout) {
            errorCode = robot_move_as::RobotMoveResult::TIMEOUT;
        }
        return finished_before_timeout && goal_success_;
    }
    return true;
}


bool RobotMove::discard_grasped_part_Q1(double timeout) {
    robot_move_as::RobotMoveGoal goal;
    goal.type = robot_move_as::RobotMoveGoal::DISCARD_GRASPED_PART_Q1;
    goal.timeout = timeout;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout;
        if (timeout == 0) {
            finished_before_timeout = ac.waitForResult();
        } else {
            finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
        }
        if (!finished_before_timeout) {
            errorCode = robot_move_as::RobotMoveResult::TIMEOUT;
        }
        return finished_before_timeout && goal_success_;
    }
    return true;    
}

bool RobotMove::discard_grasped_part_Q2(double timeout) {
    robot_move_as::RobotMoveGoal goal;
    goal.type = robot_move_as::RobotMoveGoal::DISCARD_GRASPED_PART_Q2;
    goal.timeout = timeout;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout;
        if (timeout == 0) {
            finished_before_timeout = ac.waitForResult();
        } else {
            finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
        }
        if (!finished_before_timeout) {
            errorCode = robot_move_as::RobotMoveResult::TIMEOUT;
        }
        return finished_before_timeout && goal_success_;
    }
    return true;    
}

bool RobotMove::move(Part part, Part destination, double timeout) {
    robot_move_as::RobotMoveGoal goal;
    goal_success_ = false;
    goal.type = robot_move_as::RobotMoveGoal::MOVE;
    goal.timeout = timeout;
    goal.sourcePart = part;
    goal.targetPart = destination;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout;
        if (timeout == 0) {
            finished_before_timeout = ac.waitForResult();
        } else {
            finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
        }
        ros::spinOnce();
        if (!finished_before_timeout) {
            errorCode = robot_move_as::RobotMoveResult::TIMEOUT;
        }
        return finished_before_timeout && goal_success_;
    }
    return true;
}


bool RobotMove::grasp(double timeout) {
    robot_move_as::RobotMoveGoal goal;
    goal.type = robot_move_as::RobotMoveGoal::GRASP;
    goal.timeout = timeout;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout;
        if (timeout == 0) {
            finished_before_timeout = ac.waitForResult();
        } else {
            finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
        }
        if (!finished_before_timeout) {
            errorCode = robot_move_as::RobotMoveResult::TIMEOUT;
        }
        return finished_before_timeout && goal_success_;
    }
    return true;
}

bool RobotMove::release(double timeout) {
    robot_move_as::RobotMoveGoal goal;
    goal.type = robot_move_as::RobotMoveGoal::RELEASE;
    goal.timeout = timeout;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout;
        if (timeout == 0) {
            finished_before_timeout = ac.waitForResult();
        } else {
            finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
        }
        if (!finished_before_timeout) {
            errorCode = robot_move_as::RobotMoveResult::TIMEOUT;
        }
        return finished_before_timeout && goal_success_;
    }
    return true;
}

bool RobotMove::isGripperAttached() {
    robot_move_as::RobotMoveGoal goal;
    goal.type = robot_move_as::RobotMoveGoal::IS_ATTACHED;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout = ac.waitForResult();
        if (!finished_before_timeout)
            errorCode = robot_move_as::RobotMoveResult::TIMEOUT;
        return finished_before_timeout && goal_success_;
    }
    return true;
}

/*
bool RobotMove::getRobotState(RobotState &robotState) {
    RobotMoveGoal goal;
    goal.type = RobotMoveGoal::GET_ROBOT_STATE;
    sendGoal(goal);
    if (!async_mode) {
        bool finished_before_timeout = ac.waitForResult();
        if (!finished_before_timeout)
            errorCode = RobotMoveResult::TIMEOUT;
        return finished_before_timeout && goal_success_;
    }
    return true;
}
vector<double> RobotMove::getJointsState() {
    RobotState robotState;
    bool goal_success_ = getRobotState(robotState);
    return robotState.jointStates;
}
*/
void RobotMove::sendGoal(robot_move_as::RobotMoveGoal goal) {
    action_server_returned_ = false;
    ac.sendGoal(goal, boost::bind(&RobotMove::doneCb, this, _1, _2), boost::bind(&RobotMove::activeCb, this), boost::bind(&RobotMove::feedbackCb, this, _1));
}

void RobotMove::cancel() {
    ac.cancelGoal();
}

void RobotMove::activeCb() {
    // ROS_INFO("Goal sent");
}

/*
void RobotMove::showJointState(vector<string> joint_names, vector<double> joint_values) {
    ROS_INFO("Current joints state: {");
    for (int i = 0; i < joint_names.size(); i++) {
        cout << "    [" << i + 1 << "] " << joint_names[i] << ":   " <<	joint_values[i] << ",\n";
    }
    cout << "}" << endl;
}
*/
void RobotMove::doneCb(const actionlib::SimpleClientGoalState &state, const robot_move_as::RobotMoveResultConstPtr &result) {
    action_server_returned_ = true;
    goal_success_ = result->success;
    errorCode = result->errorCode;
    //currentRobotState = result->robotState;
//    ROS_INFO("Action finished in state [%s]: %s with error code: %d",
//             state.toString().c_str(), goal_success_?"success":"failed", errorCode);
//    ROS_INFO("Gripper position is: %f, %f, %f\n",
//             currentRobotState.gripperPose.pose.position.x, currentRobotState.gripperPose.pose.position.y,
//             currentRobotState.gripperPose.pose.position.z);
//    showJointState(currentRobotState.jointNames, currentRobotState.jointStates);
}

void RobotMove::feedbackCb(const robot_move_as::RobotMoveFeedbackConstPtr &feedback) {
    //do nothing
    //currentRobotState = feedback->robotState;
}

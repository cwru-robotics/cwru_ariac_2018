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

//only uses location code of part
bool RobotMove::move_cruise_pose(Part part, double timeout) {
    ROS_INFO_STREAM("requesting move to cruise pose for part "<<part);
    robot_move_as::RobotMoveGoal goal;
    goal.type = robot_move_as::RobotMoveGoal::TO_CRUISE_POSE;
    goal.timeout = 0; //timeout;
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

/*
bool RobotMove::move_hover_pose(Part part, double timeout) {
    ROS_INFO_STREAM("requesting move to hover pose for part "<<part);
    robot_move_as::RobotMoveGoal goal;
    goal.type = robot_move_as::RobotMoveGoal::TO_CRUISE_POSE;
    goal.timeout = 0; //timeout;
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
*/

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
    ROS_INFO("waiting for goal to complete...");
    return wait_for_goal(timeout);
    /*
    if (!async_mode) {
        bool finished_before_timeout;
        if (timeout == 0) {
            finished_before_timeout = ac.waitForResult();
        } else {
            finished_before_timeout = ac.waitForResult(ros::Duration(timeout + time_tolerance));
        }
        if (!finished_before_timeout) {
            ROS_WARN("RobotMove: place part did not finish before timeout");
            errorCode = robot_move_as::RobotMoveResult::TIMEOUT;
        }
        bool robot_move_status = finished_before_timeout && goal_success_;
        ROS_INFO(";RobotMove status = %d",robot_move_status);        
        return robot_move_status;
    }*/
    //return true;
}


bool RobotMove::test_is_pickable(Part part) {
    robot_move_as::RobotMoveGoal goal;
    double timeout=2.0;
    goal.type = robot_move_as::RobotMoveGoal::TEST_IS_PICKABLE;
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
    
    
    bool RobotMove::test_is_placeable(Part destination) {
     robot_move_as::RobotMoveGoal goal;
     double timeout=2.0;
    goal.type = robot_move_as::RobotMoveGoal::TEST_IS_PLACEABLE;
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
        bool robot_move_status = finished_before_timeout && goal_success_;
        ROS_INFO(";RobotMove status = %d",robot_move_status);
        return robot_move_status;
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

//objective: return as soon as task is complete, or when hit wait expiration 
//success is returns true,  failure returns false
//make this a blocking fnc
bool RobotMove::release_placed_part(double timeout) {
    robot_move_as::RobotMoveGoal goal;
    goal.type = robot_move_as::RobotMoveGoal::RELEASE_PLACED_PART;
    goal.timeout = timeout;
    sendGoal(goal);
    return wait_for_goal(timeout);
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
    goal_start_time_ = ros::Time::now();
    ac.sendGoal(goal, boost::bind(&RobotMove::doneCb, this, _1, _2), boost::bind(&RobotMove::activeCb, this), boost::bind(&RobotMove::feedbackCb, this, _1));
}

void RobotMove::cancel() {
    ac.cancelGoal();
}

void RobotMove::activeCb() {
    // ROS_INFO("Goal sent");
}

//maybe something wrong with ac.wait fnc; make my own
bool RobotMove::ac_timed_out(double wait_time) {
    ros::Time time_now = ros::Time::now();
    double wait_time_sec = (time_now-goal_start_time_).toSec();
    if (wait_time_sec>wait_time) return true;
    else return false;
}

bool  RobotMove::wait_for_goal(double expiration_time) {
    double dt = 0.1;
    ros::Duration sleep_timer(dt);
    ros::Time start_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    double current_waiting_time = 0.0;
    //bool keep_polling = true;
    while (current_waiting_time< expiration_time+2.0) //tolerance on expiration time
    {
        if (action_server_returned_) {
            if(goal_success_) { ROS_INFO("action server returned success==true"); }
            else { ROS_WARN("action server returned success==false");  }
            return goal_success_; //if goal done, return "success" true/false
             
        }
        if (ac_timed_out(expiration_time) ) {
            ROS_WARN("action server did not complete task within expiration-time = %f",expiration_time);
            return false;   
        }
        current_time = ros::Time::now();
        current_waiting_time = (current_time-start_time).toSec();        
        sleep_timer.sleep();
    }
    ROS_WARN("something is very wrong--waiting time> expiration time for action; should not happen");
    return false;
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
    ROS_INFO("Action finished  with result error code: %d", (int) errorCode);
//    ROS_INFO("Gripper position is: %f, %f, %f\n",
//             currentRobotState.gripperPose.pose.position.x, currentRobotState.gripperPose.pose.position.y,
//             currentRobotState.gripperPose.pose.position.z);
//    showJointState(currentRobotState.jointNames, currentRobotState.jointStates);
}

void RobotMove::feedbackCb(const robot_move_as::RobotMoveFeedbackConstPtr &feedback) {
    //do nothing
    //currentRobotState = feedback->robotState;
}

//
//  wsn April 2018
//

#include <conveyor_as/ConveyorInterface.h>
ConveyorInterface::ConveyorInterface(ros::NodeHandle* nodehandle) : nh_(*nodehandle),ac("conveyor_as", true) {

    ROS_INFO("waiting for conveyor action server: ");
    bool server_exists = ac.waitForServer(ros::Duration(1.0));
    while (!server_exists) {
        ROS_WARN("waiting conveyor action server...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        server_exists = ac.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to conveyor action server"); // if here, then we connected to the server;   

}

void ConveyorInterface::sendGoal(unsigned short int goal_code) {
    conveyorGoal_.conveyor_task_code = goal_code;
    action_server_returned_ = false;
    box_status_ = conveyor_as::conveyorResult::CONVEYOR_IN_MOTION;
    ac.sendGoal(conveyorGoal_, boost::bind(&ConveyorInterface::doneCb, this, _1, _2), boost::bind(&ConveyorInterface::activeCb, this), boost::bind(&ConveyorInterface::feedbackCb, this, _1));

}

void ConveyorInterface::activeCb() {
    // ROS_INFO("Goal sent");
}

void ConveyorInterface::move_new_box_to_Q1() {
    sendGoal(conveyor_as::conveyorGoal::MOVE_NEW_BOX_TO_Q1_STATION);
    }
void ConveyorInterface::move_box_Q1_to_Q2(){
    sendGoal(conveyor_as::conveyorGoal::MOVE_BOX_Q1_TO_Q2_STATION);
    }
void ConveyorInterface::move_box_Q1_to_drone_depot(){
    sendGoal(conveyor_as::conveyorGoal::MOVE_BOX_Q1_TO_DRONE_DEPOT);
    }
void ConveyorInterface::move_box_Q2_to_drone_depot(){
    sendGoal(conveyor_as::conveyorGoal::MOVE_BOX_Q2_TO_DRONE_DEPOT);
    }
void ConveyorInterface::move_lead_box_to_drone_depot(){
    sendGoal(conveyor_as::conveyorGoal::MOVE_LEAD_BOX_TO_DRONE_DEPOT);   
    } //if box is past Q2, but still on conveyor


void ConveyorInterface::doneCb(const actionlib::SimpleClientGoalState &state, const conveyor_as::conveyorResultConstPtr &result) {
    box_status_ = result->box_status;
    ROS_INFO("Action finished  with result code: %d",  (int) box_status_);
    action_server_returned_ = true;    
}

void ConveyorInterface::feedbackCb(const conveyor_as::conveyorFeedbackConstPtr &feedback) {
    estimated_seconds_to_goal= feedback->estimated_seconds_to_goal;
    box1_distance = feedback->box1_distance;
    box2_distance = feedback->box2_distance;
    box3_distance = feedback->box3_distance;
    sensors_are_active = feedback->sensors_are_active;
    drone_depot_sensor_sees_box = feedback->drone_depot_sensor_sees_box;
    if (drone_depot_sensor_sees_box) ROS_WARN("drone_depot_sensor_sees_box");
    ROS_INFO("fdbk: est seconds to goal = %f",estimated_seconds_to_goal);
    if (sensors_are_active) ROS_INFO("sensors are active");
    else ROS_INFO("sensors not publishing");
}


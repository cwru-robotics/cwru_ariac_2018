//
// created by wsn, april 2018
//

#ifndef CONVEYOR_INTERFACE_H
#define CONVEYOR_INTERFACE_H

#include <map>
#include <string>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
using namespace std;

#include <conveyor_as/conveyorAction.h>  

class ConveyorInterface {
public:
    ConveyorInterface(ros::NodeHandle* nodehandle);

    bool action_server_returned() {
        return action_server_returned_;
    };

    unsigned short int get_box_status() {
        return box_status_;
    };
    bool get_action_server_returned() {
        return action_server_returned_;
    }
    
    bool drone_depot_sees_box() {return drone_depot_sensor_sees_box;};

    void move_new_box_to_Q1();
    void move_box_Q1_to_Q2();
    void move_box_Q1_to_drone_depot();
    void move_box_Q2_to_drone_depot();
    void move_lead_box_to_drone_depot(); //if box is past Q2, but still on conveyor
    double estimated_seconds_to_goal;
    double box1_distance;
    double box2_distance;
    double box3_distance;
    bool sensors_are_active;    
    bool drone_depot_sensor_sees_box;
    
protected:

    ros::NodeHandle nh_;

    actionlib::SimpleActionClient<conveyor_as::conveyorAction> ac;
    conveyor_as::conveyorGoal conveyorGoal_;

    bool action_server_returned_;
    unsigned short int box_status_;

    void sendGoal(unsigned short int goal_code);
    void doneCb(const actionlib::SimpleClientGoalState& state, const conveyor_as::conveyorResultConstPtr& result);
    void feedbackCb(const conveyor_as::conveyorFeedbackConstPtr& feedback);
    void activeCb();
};


#endif //CONVEYOR_INTERFACE_H

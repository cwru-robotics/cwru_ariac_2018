// example_conveyor_action_client: 
// wsn, April 2018

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include<conveyor_as/conveyorAction.h>

void doneCb(const actionlib::SimpleClientGoalState& state,
        const conveyor_as::conveyorResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    int box_status = result->box_status;
    ROS_INFO("got result box_status = %d",box_status);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_conveyor_action_client_node"); // name this node 

    //preamble: need to instantiate a goal object and an action client, and connect it to the conveyor server
    conveyor_as::conveyorGoal goal;

    // use the name of our server, which is: conveyor_as (named in conveyor_as.cpp)
    // the "true" argument says that we want our new client to run as a separate thread (a good idea)
    actionlib::SimpleActionClient<conveyor_as::conveyorAction> conveyor_client("conveyor_as", true);

    // attempt to connect to the server:
    ROS_INFO("waiting for conveyor server: ");
    bool server_exists = false;
    while (!server_exists) {
        ROS_WARN("waiting conveyor server...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        server_exists = ac.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to conveyor action server"); // if here, then we connected to the server;   




    while (true) {
        // stuff a goal message:
        goal.conveyor_task_code = conveyor_as::conveyorGoal::MOVE_NEW_BOX_TO_Q1_STATION; 
        action_client.sendGoal(goal, &doneCb, &feedbackCb); // we could also name additional callback functions here, if desired
        //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this

        bool finished_before_timeout = action_client.waitForResult(ros::Duration(5.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result for goal number %d", g_count);
            return 0;
        } else {
            //if here, then server returned a result to us
        }

    }

    return 0;
}


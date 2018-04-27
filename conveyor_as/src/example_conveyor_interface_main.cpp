// example_conveyor_action_client: 
// wsn, April 2018

#include<ros/ros.h>
#include<conveyor_as/ConveyorInterface.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "example_conveyor_interface_main"); // name this node 
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    //instantiate a conveyor interface object:
    ROS_INFO("instantiating a conveyorInterface");
    ConveyorInterface conveyorInterface(&nh);


    //the following illustrates invoking some of the conveyor functions
    int ans;
    ROS_INFO("conveyor interface tester:");
    bool done=false;
    while (true) {
        ROS_INFO("enter 1 to move box to Q1, 2 to  move box from Q1 to Q2, 3 to move box Q2 to depot");
        cin>>ans;
        switch(ans) {
          case 1:
             conveyorInterface.move_new_box_to_Q1();
             //poll to see if done;
             while (!done) {
                ros::spinOnce();
                done = conveyorInterface.action_server_returned();
             }
          break;
          case 2:
             conveyorInterface.move_box_Q1_to_Q2();
          break;
          case 3:
             conveyorInterface.move_box_Q2_to_drone_depot();
          break;

          default: 
             ROS_WARN("input not recognized");

        }
      }



    return 0;
}


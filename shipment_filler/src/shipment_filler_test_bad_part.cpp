//shipment_filler

#include<order_manager/order_manager.h>
#include<shipment_filler/ShipmentFiller.h>
#include <robot_move_as/RobotMove.h>
#include <robot_move_as/RobotMoveActionServer.h>

int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "shipment_filler"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    //ROS_INFO("main: instantiating an object of type OrderManager");
    //OrderManager orderManager(&nh);
    ShipmentFiller shipmentFiller(&nh);
    int ans;

    inventory_msgs::Part pick_part,place_part;
    RobotMove robotMove(&nh);
    robotMove.enableAsync();
    //bool RobotMove::toPredefinedPose(int8_t predefined_pose_code, double timeout)
    //robotMove.toPredefinedPose(robot_move_as::RobotMoveGoal::QUAL_SENSOR_1_CRUISE_POSE);
    //ros::Duration(2.0).sleep();
    //robotMove.toPredefinedPose(robot_move_as::RobotMoveGoal::QUAL_SENSOR_1_HOVER_POSE);
    //ros::Duration(2.0).sleep();

    bool found_bad_part = false;
    ROS_INFO("waiting to see bad part: ");
    while (!found_bad_part) {
        ros::spinOnce();
        found_bad_part = shipmentFiller.get_bad_part_Q1(pick_part);
    }
    ROS_INFO_STREAM("found bad part: "<<pick_part<<endl);
    // this is how to command the robot action server
    robotMove.pick(pick_part);
    while (!robotMove.actionFinished())
    {  
        ROS_INFO("waiting for result");
        ros::Duration(1).sleep();
    }
    robotMove.place_part_no_release(pick_part);
    while (!robotMove.actionFinished())
    {  
        ROS_INFO("waiting for result");
        ros::Duration(0.5).sleep();
    }
    
    found_bad_part = false;
    ROS_INFO("waiting to see bad part: ");
    while (!found_bad_part) {
        ros::Duration(0.2).sleep();
        ros::spinOnce();        
        found_bad_part = shipmentFiller.get_bad_part_Q1(pick_part);
    }
    ROS_INFO("found bad part");
    //cout<<"enter 1: ";
    //cin>>ans;
    ROS_INFO("discarding grasped part");
    robotMove.discard_grasped_part_Q1(2.0);
    while (!robotMove.actionFinished())
    {  
        ROS_INFO("waiting for result");
        ros::Duration(0.5).sleep();
    }    
    

}

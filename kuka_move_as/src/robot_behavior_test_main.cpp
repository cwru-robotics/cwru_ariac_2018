//example using RobotBehaviorInterface library

#include <kuka_move_as/RobotBehaviorInterface.h>
//#include <kuka_move_as/KukaBehaviorActionServer.h>  //do I  need this?


void set_part_vals(Part &pick_part,Part &place_part) {
    geometry_msgs::PoseStamped pick_pose,place_pose;
    pick_pose.header.frame_id="world";
    place_pose.header.frame_id="world";
    
    //a hard-coded gear source location: 
    pick_pose.pose.position.x = -0.8250; //0.341; //-0.2; //-0.369; //Translation: [-0.369, -0.597, 0.726]
    pick_pose.pose.position.y = 1.0475; //3.155; //0.33; //-0.597;
    pick_pose.pose.position.z = 0.9675359; //0.7555; //0.725; //0.726;    
    pick_pose.pose.orientation.x=0;
    pick_pose.pose.orientation.y=0.1246747288630;
    pick_pose.pose.orientation.z= 0;
    pick_pose.pose.orientation.w=0.99219;    

    pick_part.name="gear_part"; //pulley_part"; //gasket_part"; //gear_part";
    pick_part.location= inventory_msgs::Part::BIN3; //
    pick_part.pose = pick_pose;  
    
    
    //    e.g., gear part to be placed centered in box0:
    //  x: 1.16012185522
    //  y: 0.900165848419
    //  z: 0.444559999995
    place_pose.pose.position.x = 1.16012185522; //0.23; //0;//0.300; //AGV1 frame: Translation: [0.300, 3.300, 0.750]
    place_pose.pose.position.y = 0.900165848419; //+= 0.01; //= 3.13; //0; //3.300;
    place_pose.pose.position.z = 0.444559999995; //0; //0.750;    
    place_pose.pose.orientation.x=0;
    place_pose.pose.orientation.y=0;
    place_pose.pose.orientation.z= 0;
    place_pose.pose.orientation.w=1;    

    place_part = pick_part;  //copy over data, replace what needs to  be  replaced
    place_part.pose = place_pose;
    place_part.location= inventory_msgs::Part::QUALITY_SENSOR_1; 
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_behavior_tester"); //name this node
    ros::NodeHandle nh;
    inventory_msgs::Part pick_part,place_part;
    ROS_INFO("instantiating a robotBehaviorInterface");
    RobotBehaviorInterface robotBehaviorInterface(&nh);
    
    //set_part_vals(pick_part,place_part); //populate with test vals
    /*
    ROS_INFO("attempting pick command, BIN1");
    pick_part.location= inventory_msgs::Part::BIN1;
    robotBehaviorInterface.pick(pick_part);
    */
    ROS_INFO("attempting pick command, BIN2");
    pick_part.location= inventory_msgs::Part::BIN2;
    robotBehaviorInterface.pick(pick_part);

    /*
    ROS_INFO("attempting pick command, BIN3");
    pick_part.location= inventory_msgs::Part::BIN3;
    robotBehaviorInterface.pick(pick_part);

    ROS_INFO("attempting pick command, BIN4");
    pick_part.location= inventory_msgs::Part::BIN4;
    robotBehaviorInterface.pick(pick_part);

    //BIN 5 is broken!!

    //ROS_INFO("attempting pick command, BIN5");
    //pick_part.location= inventory_msgs::Part::BIN5;
    //robotBehaviorInterface.pick(pick_part);
    */
    ROS_INFO("attempting pick command, Q1");
    pick_part.location= inventory_msgs::Part::QUALITY_SENSOR_1;
    robotBehaviorInterface.pick(pick_part);
    
    ROS_INFO("attempting discard from Q1");
    robotBehaviorInterface.discard_grasped_part();
    
    ROS_INFO("client is done");
    return 0;

}




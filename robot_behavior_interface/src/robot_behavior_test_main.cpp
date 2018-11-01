//example using RobotBehaviorInterface library

#include <robot_behavior_interface/RobotBehaviorInterface.h>


void set_part_vals(Part &pick_part,Part &place_part) {
    geometry_msgs::PoseStamped pick_pose,place_pose;
    pick_pose.header.frame_id="world";
    place_pose.header.frame_id="world";
    /*  here is a piston-rod part in BIN2:
           frame_id: world
        pose: 
          position: 
            x: -0.775109
            y: 0.320213
            z: 0.763033
          orientation: 
            x: 0.0416977
            y: 0.0933736
            z: 0.38273
            w: 0.918183
    */
    //a hard-coded gear source location: 
    pick_pose.pose.position.x = -0.775109; //0.341; //-0.2; //-0.369; //Translation: [-0.369, -0.597, 0.726]
    pick_pose.pose.position.y = 0.320213; //3.155; //0.33; //-0.597;
    pick_pose.pose.position.z = 0.763033; //0.7555; //0.725; //0.726;    
    pick_pose.pose.orientation.x=0.0416977;
    pick_pose.pose.orientation.y=0.0933736;
    pick_pose.pose.orientation.z= 0.38273;
    pick_pose.pose.orientation.w=0.918183;    

    pick_part.name="piston_rod_part"; //pulley_part"; //gasket_part"; //gear_part";
    pick_part.location= inventory_msgs::Part::BIN2; //
    pick_part.pose = pick_pose;  
    
    
    //    e.g., gear part to be placed centered in box0:
    //  x: 1.16012185522
    //  y: 0.900165848419
    //  z: 0.444559999995
    place_pose.pose.position.x = 0.5822; //0.23; //0;//0.300; //AGV1 frame: Translation: [0.300, 3.300, 0.750]
    place_pose.pose.position.y = 0.8688; //+= 0.01; //= 3.13; //0; //3.300;
    place_pose.pose.position.z = 0.5873; //0; //0.750;    
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
    
    set_part_vals(pick_part,place_part); //populate with test vals
    /*
    ROS_INFO("attempting pick command, BIN1");
    pick_part.location= inventory_msgs::Part::BIN1;
    robotBehaviorInterface.pick(pick_part);
    */
    ROS_INFO_STREAM("attempting pick command for part "<<pick_part<<endl);
    //pick_part.location= inventory_msgs::Part::BIN2;
    robotBehaviorInterface.pick_part_from_bin(pick_part); 
    
    ROS_INFO("attempting discard");
    robotBehaviorInterface.discard_grasped_part(pick_part);    

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
    /*
    ROS_INFO("attempting pick command, Q1");
    pick_part.location= inventory_msgs::Part::QUALITY_SENSOR_1;
    robotBehaviorInterface.pick(pick_part);
    
    ROS_INFO("attempting discard from Q1");
    robotBehaviorInterface.discard_grasped_part();
    */
    ROS_INFO("client is done");
    return 0;

}




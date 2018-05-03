//test fnc: user provides x and y, robot  goes through motions for bin1

//#include<order_manager/order_manager.h>
//#include<shipment_filler/ShipmentFiller.h>

#include <kuka_move_as/RobotBehaviorInterface.h>
#include<bin_inventory/bin_inventory.h>
#include <inventory_msgs/Part.h>
int ans;


int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "pick_from_inventory_tester"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    //ROS_INFO("main: instantiating an object of type OrderManager");
    //OrderManager orderManager(&nh);

    RobotBehaviorInterface robotBehaviorInterface(&nh);
    
    inventory_msgs::Part pick_part;

    
  geometry_msgs::Pose part_target_pose;
  part_target_pose.position.x = 0.1;
  part_target_pose.position.y = -0.1;
  part_target_pose.position.z = 0.0;

  part_target_pose.orientation.x = 0.0;
  part_target_pose.orientation.y = 0.0;
  part_target_pose.orientation.z = 0.0;
  part_target_pose.orientation.w = 1.0;    
  
  pick_part.name = "piston_rod_part";  
  pick_part.location =  inventory_msgs::BIN1;
  pick_part.pose.pose = part_target_pose;
  pick_part.pose.header.frame_id = "world";
  

    ros::Time start_time;
    ros::Time end_time;
    double x,y;
    double pick_time;
    while(true) {
    //for (int i_part_type=1;i_part_type<=NUM_PART_TYPES;i_part_type++) {
    
            cout<<"enter test x val: ";
            cin>>x;
            cout<<"enter test y val: ";
            cin>>y;         
            part_target_pose.position.x = x;
            part_target_pose.position.y = y;
            //part_target_pose.position.z = f(x);
            start_time = ros::Time::now();

            pick_part.pose.pose = part_target_pose;

              
              ROS_INFO("attempting pick...");
               if(robotBehaviorInterface.pick_part_from_bin(pick_part)) { 
                  end_time = ros::Time::now();
                  pick_time = (end_time - start_time).toSec();
                  ROS_INFO("pick time = %f",pick_time);
               }                  
                      
        }

}

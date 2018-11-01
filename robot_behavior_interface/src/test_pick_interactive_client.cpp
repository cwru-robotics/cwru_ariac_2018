//test fnc: user provides x and y, robot  goes through motions for bin1

//#include<order_manager/order_manager.h>
//#include<shipment_filler/ShipmentFiller.h>

#include <robot_behavior_interface/RobotBehaviorInterface.h>
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
  part_target_pose.orientation.y = 0.1;
  part_target_pose.orientation.z = 0.0;
  part_target_pose.orientation.w = 0.995;    
  
  pick_part.name = "piston_rod_part";  
  pick_part.location =  inventory_msgs::Part::BIN1;
  pick_part.pose.pose = part_target_pose;
  pick_part.pose.header.frame_id = "world";
  

    ros::Time start_time;
    ros::Time end_time;
    double x,y,z;
    double pick_time;
    int position_code;
    while(true) {
    //for (int i_part_type=1;i_part_type<=NUM_PART_TYPES;i_part_type++) {
        cout<<"enter position code, 1 through 9: ";
        cin>>position_code;
        switch(position_code) {
            //check how deep can reach with "near" strategy:  
            case 1: //left front
                part_target_pose.position.x = -0.67; //-0.63;
                part_target_pose.position.y = -0.645;
                part_target_pose.position.z = 0.735;                
                break;
            case 2: //middle front
                part_target_pose.position.x = -0.67; //-0.63;
                part_target_pose.position.y = -0.468;
                part_target_pose.position.z = 0.735;                
                break;
            case 3: //right front
                part_target_pose.position.x = -0.65; //-0.63; //-0.69 FAILS--hits right upright
                part_target_pose.position.y = -0.325; //y-limit to avoid hitting right edge of bin; 
                part_target_pose.position.z = 0.735;                
                break;                
            case 4: //left mid
                part_target_pose.position.x =  -0.6; //-0.78;
                part_target_pose.position.y = -0.645;
                part_target_pose.position.z = 0.7625;                
                break;
            case 5: //middle ctr
                part_target_pose.position.x =  -0.6; //-0.78;
                part_target_pose.position.y = -0.468;
                part_target_pose.position.z = 0.7625;                
                break;
            case 6: //right mid
                part_target_pose.position.x =  -0.6; //-0.78; //TRY CLOSER
                part_target_pose.position.y = -0.328; //-0.29; //hits at -0.29
                part_target_pose.position.z = 0.7625;                
                break; 
            case 7: //left rear
                part_target_pose.position.x =  -0.84; //see what  happens closer to middle  row
                part_target_pose.position.y = -0.645;
                part_target_pose.position.z = 0.790;                
                break;
            case 8: //middle rear
                part_target_pose.position.x =  -0.84;
                part_target_pose.position.y = -0.468;
                part_target_pose.position.z = 0.790;                
                break;
            case 9: //right rear
                part_target_pose.position.x =  -0.84;
                part_target_pose.position.y = -0.29; //at closer x, cannot reach larger y
                part_target_pose.position.z = 0.790;                
                break;                 
        }

            //z = 0.735+ -(0.055/0.12)*(x+0.78);
            //ROS_INFO("x,y,z = %f, %f %f",x,y,z);
            //part_target_pose.position.z = f(x);
            start_time = ros::Time::now();

            pick_part.pose.pose = part_target_pose;

              
              ROS_INFO("attempting pick...");
               if(robotBehaviorInterface.test_pick_part_from_bin(pick_part)) { 
                  end_time = ros::Time::now();
                  pick_time = (end_time - start_time).toSec();
                  ROS_INFO("pick time = %f",pick_time);
               }                  
                      
        }

}

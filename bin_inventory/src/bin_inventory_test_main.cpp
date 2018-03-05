#include<bin_inventory/bin_inventory.h>

int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "example_inventory_test_main"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type BinInventory");
    BinInventory binInventory(&nh);  

    while(ros::ok()) {
      binInventory.update();
      ros::Duration(1.0).sleep();
    }

    return 0;
} 


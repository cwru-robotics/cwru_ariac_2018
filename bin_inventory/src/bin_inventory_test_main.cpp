#include<bin_inventory/bin_inventory.h>
int ans;

int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "example_inventory_test_main"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type BinInventory");
    BinInventory binInventory(&nh);  

    binInventory.update();
    inventory_msgs::Inventory inventory_msg;
    binInventory.get_inventory(inventory_msg);
    ROS_INFO_STREAM(inventory_msg<<endl);
    std::string part_name("piston_rod_part");
    int part_id = name_to_part_id_mappings[part_name];

    int bin_num, partnum;
    geometry_msgs::PoseStamped part_pose;
    //FIX ME! changed find_part()
    while (binInventory.find_part(part_name,bin_num,part_pose, partnum)) {
        binInventory.remove_part_from_inventory(part_id, partnum);
        binInventory.get_inventory(inventory_msg);
        ROS_INFO_STREAM(inventory_msg<<endl);      
        cout<<"enter 1: ";
        cin>>ans;
    }

   

    return 0;
} 


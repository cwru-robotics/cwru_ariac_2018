//test fnc: discard ALL parts in bin inventory!!

//#include<order_manager/order_manager.h>
//#include<shipment_filler/ShipmentFiller.h>

#include <robot_behavior_interface/RobotBehaviorInterface.h>
#include<bin_inventory/bin_inventory.h>
int ans;

int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "pick_from_inventory_tester"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    //ROS_INFO("main: instantiating an object of type OrderManager");
    //OrderManager orderManager(&nh);

    RobotBehaviorInterface robotBehaviorInterface(&nh);

    inventory_msgs::Part pick_part, place_part;
    inventory_msgs::Inventory inventory_msg;

    ROS_INFO("main: instantiating an object of type BinInventory");
    BinInventory binInventory(&nh);
    binInventory.update();

    //void BinInventory::get_inventory(inventory_msgs::Inventory &inventory_msg) {
    binInventory.get_inventory(inventory_msg);
    binInventory.print_inventory_msg();

    /*     part.location = bin_num;
     part.pose = part_pose;
     part.name = part_name.c_str();*/
    //start from 2--skip the gear parts
    //start from 3--begin w/ flanges
    ros::Time start_time;
    ros::Time end_time;
    double pick_time;
    //for (int i_part_type=1;i_part_type<=NUM_PART_TYPES;i_part_type++) {
    //modify loop below to focus on specific parts
    for (int i_part_type = 1; i_part_type <= NUM_PART_TYPES; i_part_type++) {

        std::string part_name = part_id_to_name_mappings[i_part_type];
        ROS_INFO_STREAM("attempting to remove parts  of  type " << part_name << endl);
        int nparts = binInventory.num_parts(i_part_type);
        ROS_INFO_STREAM("found " << nparts << " of these" << endl);
        if (nparts > 0) {
            for (int ipart = 0; ipart < nparts; ipart++) {
                ROS_INFO("removing item %d of part_id %d", ipart, i_part_type);
                cout << "enter 1: ";
                cin>>ans;
                start_time = ros::Time::now();
                unsigned short int bin_num = inventory_msg.inventory[i_part_type].bins[ipart];
                geometry_msgs::PoseStamped part_pose = inventory_msg.inventory[i_part_type].part_stamped_poses[ipart];
                ROS_INFO_STREAM("found part in bin " << bin_num << " at pose " << part_pose << endl);
                pick_part.location = bin_num;
                pick_part.pose = part_pose;
                pick_part.name = part_name.c_str();
                ROS_INFO_STREAM("pick_part: " << pick_part << endl);


                ROS_INFO("attempting pick...");
                if (robotBehaviorInterface.pick_part_from_bin(pick_part)) {
                    ROS_INFO("dropping part");
                    robotBehaviorInterface.release();
                    end_time = ros::Time::now();
                    pick_time = (end_time - start_time).toSec();
                    ROS_INFO("pick time = %f", pick_time);
                }

            }
        }
    }
    ROS_INFO("done with all inventory; returning");
    return 0;

}



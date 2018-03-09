//shipment_filler

#include<order_manager/order_manager.h>
#include<shipment_filler/ShipmentFiller.h>
#include <robot_move_as/RobotMove.h>
#include <robot_move_as/RobotMoveActionServer.h>

int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "shipment_filler"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    ROS_INFO("main: instantiating an object of type OrderManager");
    OrderManager orderManager(&nh);
    ShipmentFiller shipmentFiller(&nh);
    RobotMove robotMove(&nh);
    
    inventory_msgs::Part pick_part,place_part;
    inventory_msgs::Inventory inventory_msg;
    osrf_gear::Shipment shipment;
    bool got_shipment = false;
    bool successfully_filled_order = false;
    bool init_pack_shipment= false;
    bool replaced_faulty_parts=false;
    bool adjusted_part_locations=false;
    //bool corrected_dropped_part=false;
    bool reported_shipment_to_drone=false;
    bool advanced_shipment_on_conveyor=false;

    //send robot to waiting pose
    int ans;
    ROS_INFO("get a box into position: ");
    advanced_shipment_on_conveyor= 
                shipmentFiller.advance_shipment_on_conveyor(BOX_INSPECTION1_LOCATION_CODE);
    //cout<<"enter 1: ";
    //cin>>ans;
    while (ros::ok()) {
        orderManager.update_inventory();
        //sort all the order messages queued in callback
        //NOTE: with this construction, an incoming order NEVER pre-empts
        //a current shipment in progress; CAN pre-empt an order in progress,
        // but current shipment will be completed
        for (int i = 0; i < 10; i++) { 
            ros::spinOnce();
        }

        orderManager.get_inventory(inventory_msg);
        ROS_INFO("test_main printing inventory");

        orderManager.print_inventory_succinct(inventory_msg);

        got_shipment = orderManager.choose_shipment(shipment);
        successfully_filled_order = false;
        if (!got_shipment) {
          ROS_INFO("waiting for shipment");
          ros::Duration(0.5).sleep();
        }
        else {
            ROS_INFO_STREAM("shipment to be filled: " << shipment << endl);
            //prep for drone request: set shipment name
            shipmentFiller.set_drone_shipment_name(shipment);            
            //try to fill shipment; start w/ positioning box under box camera1
            advanced_shipment_on_conveyor= 
                shipmentFiller.advance_shipment_on_conveyor(BOX_INSPECTION1_LOCATION_CODE);
            //do robot moves to fill shipment
            
            init_pack_shipment= shipmentFiller.fill_shipment(shipment);

            //advance box to first inspection station
            //ROS_INFO("box presumed filled; goto inspection station 1");
            //advanced_shipment_on_conveyor= 
            //    shipmentFiller.advance_shipment_on_conveyor(BOX_INSPECTION1_LOCATION_CODE);
            //fix any faulty parts:
            replaced_faulty_parts = shipmentFiller.replace_faulty_parts_inspec1(shipment);
            //advance shipment to next inspection  station:
            advanced_shipment_on_conveyor= 
                shipmentFiller.advance_shipment_on_conveyor(BOX_INSPECTION2_LOCATION_CODE);
            //fix any faulty parts:
            replaced_faulty_parts = shipmentFiller.replace_faulty_parts_inspec2(shipment);
            //adjust part locations in box:
            adjusted_part_locations = shipmentFiller.adjust_shipment_part_locations(shipment);
            //report shipment has been filled:
            orderManager.current_shipment_has_been_filled();
            //advance order to drone dock:
            advanced_shipment_on_conveyor= 
                shipmentFiller.advance_shipment_on_conveyor(DRONE_DOCK_LOCATION_CODE);
            //send notice to drone:
            
            reported_shipment_to_drone= shipmentFiller.report_shipment_to_drone();
            //send robot to waiting pose; update inventory
         
         }
  }

}

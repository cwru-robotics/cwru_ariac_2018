//shipment_filler

#include<order_manager/order_manager.h>
#include<shipment_filler/ShipmentFiller.h>
#include <robot_move_as/RobotMove.h>
#include <robot_move_as/RobotMoveActionServer.h>

/// Start the competition by waiting for and then calling the start ROS Service.
void start_competition(ros::NodeHandle & node) {
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client =
    node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!start_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger srv;  // Combination of the "request" and the "response".
  start_client.call(srv);  // Call the start Service.
  if (!srv.response.success) {  // If not successful, print out why.
    ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
  } else {
    ROS_INFO("Competition started!");
  }
}

int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "shipment_filler"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    //ROS_INFO("main: instantiating an object of type OrderManager");
    //OrderManager orderManager(&nh);

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
    
    ROS_INFO("moving robot to init pose");
    robotMove.toPredefinedPose(robot_move_as::RobotMoveGoal::INIT_POSE);
    int ans;
    //cout<<"enter  1: ";
    //cin>>ans;
            
    ros::Duration(2.0).sleep();
    ROS_INFO("instantiating a ShipmentFiller");
    ShipmentFiller shipmentFiller(&nh);
    ROS_INFO("updating inventory");
    shipmentFiller.update_inventory();
    
    ROS_INFO("starting the competition");
    start_competition(nh); //start the competition
    
    double competition_start_time = ros::Time::now().toSec();
        
        
    //send robot to waiting pose
    //int ans;
    ROS_INFO("get a box into position: ");
    advanced_shipment_on_conveyor= 
                shipmentFiller.advance_shipment_on_conveyor(BOX_INSPECTION1_LOCATION_CODE);
    //cout<<"enter 1: ";
    //cin>>ans;
    while (ros::ok()) {
        shipmentFiller.update_inventory();
        //sort all the order messages queued in callback
        //NOTE: with this construction, an incoming order NEVER pre-empts
        //a current shipment in progress; CAN pre-empt an order in progress,
        // but current shipment will be completed
        for (int i = 0; i < 10; i++) { 
            ros::spinOnce();
        }

        //orderManager.get_inventory(inventory_msg);
        //ROS_INFO("test_main printing inventory");

        //orderManager.print_inventory_succinct(inventory_msg);

        got_shipment = shipmentFiller.choose_shipment(shipment);
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
            //advanced_shipment_on_conveyor= 
            //    shipmentFiller.advance_shipment_on_conveyor(BOX_INSPECTION1_LOCATION_CODE);
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
            shipmentFiller.current_shipment_has_been_filled();
            //advance order to drone dock:
            advanced_shipment_on_conveyor= 
                shipmentFiller.advance_shipment_on_conveyor(DRONE_DOCK_LOCATION_CODE);
            //send notice to drone:
            
            reported_shipment_to_drone= shipmentFiller.report_shipment_to_drone();
            //send robot to waiting pose; update inventory
         
         }
  }

}

//this is a top-level program, 
// this version is only a simple prototype, but it illustrates the required steps and I/O
//this program is responsible for the logic of: obtaining orders, 
//choosing shipments to work on, choosing parts to pick from bins, making corrections to placed parts,
//checking quality sensors and part placements and making corrections, controlling the conveyor, and
// calling the drone to deliver shipments

//this program could get long.  The intent of "ShipmentFiller" is to create functions that bundle up some of
//the detail of operations, hopefully making this top-level program a reasonable length

#include<order_manager/order_manager.h>
#include<shipment_filler/ShipmentFiller.h>
#include <robot_behavior_interface/RobotBehaviorInterface.h>
#include<bin_inventory/bin_inventory.h>
#include<box_inspector/box_inspector.h>

const double COMPETITION_TIMEOUT=500.0; // need to  know what this is for the finals;
// want to ship out partial credit before time runs out!


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
    //here are a bunch of useful objects.  Most of these are also instantiated  "ShipmentFiller"
    //still deciding what and how much to expose at this top level
    int ans;
    
    ROS_INFO("main: instantiating an object of type OrderManager");
    OrderManager orderManager(&nh); //shipmentFiller also owns one of these, which is public
    ROS_INFO("instantiating a RobotBehaviorInterface");
    RobotBehaviorInterface robotBehaviorInterface(&nh); //shipmentFiller owns one as well
    ROS_INFO("instantiationg a BinInventory object");
    BinInventory binInventory(&nh);  //shipmentFiller owns one of these, which is public
    ROS_INFO("instantiating a BoxInspector");
    BoxInspector boxInspector(&nh);
    ROS_INFO("instantiating a ShipmentFiller");
    ShipmentFiller shipmentFiller(&nh);

    inventory_msgs::Part pick_part,place_part;
    inventory_msgs::Inventory current_inventory;
    osrf_gear::Shipment shipment;
    geometry_msgs::PoseStamped box_pose_wrt_world;  
    vector<osrf_gear::Model>  desired_models_wrt_world;
    osrf_gear::Model current_model;
    //inventory_msgs::Part current_part;
    int bin_num,partnum;
    geometry_msgs::PoseStamped part_pose;
    
    bool got_shipment = false;
    bool successfully_filled_order = false;
    bool init_pack_shipment= false;
    bool replaced_faulty_parts=false;
    bool adjusted_part_locations=false;
    //bool corrected_dropped_part=false;
    bool reported_shipment_to_drone=false;
    bool advanced_shipment_on_conveyor=false;
    bool go_on= true;

    ROS_INFO("attempting to get an inventory update; pretty much screwed until this is possible");
    while (!binInventory.update()) {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    
    ROS_INFO("got initial inventory");
    binInventory.get_inventory(current_inventory);
    
    ROS_INFO("starting the competition");
    start_competition(nh); //start the competition
    
    double competition_start_time = ros::Time::now().toSec(); //check the clock= start time
        
        
    //MAKE BOX CONTROL AN ACTION SERVER for multi-tasking
    ROS_INFO("getting a box into position: ");
    advanced_shipment_on_conveyor= 
                shipmentFiller.advance_shipment_on_conveyor(BOX_INSPECTION1_LOCATION_CODE);
    while(!boxInspector.get_box_pose_wrt_world(box_pose_wrt_world)) {
        ROS_INFO("trying to see box pose w/rt world...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
    ROS_INFO_STREAM("box pose wrt world is: "<<box_pose_wrt_world<<endl);

    //first box is at Q1 inspection station; get a shipment:
        //got_shipment = orderManager.choose_shipment(shipment);
        successfully_filled_order = false;
        //can't go any further until receive new shipment request, or until near competition expiration
        while(ros::ok()) {

        while (!orderManager.choose_shipment(shipment)) {
          ROS_INFO("waiting for shipment");
          ros::Duration(0.5).sleep();
        }
        //now have a shipment; start  processing it
        ROS_INFO_STREAM("shipment to be filled: " << shipment << endl);
        //prep for drone request: set shipment name
        shipmentFiller.set_drone_shipment_name(shipment);       
        boxInspector.compute_shipment_poses_wrt_world(shipment,box_pose_wrt_world,desired_models_wrt_world);

        //try to fill shipment; do robot moves to fill shipment
        int num_parts = desired_models_wrt_world.size();
        ROS_INFO("trying to fill box with %d products",num_parts);
        int i_model=0;
        int partnum_in_inventory;
        
        
        //-----------------HERE IS THE MAIN LOOP FOR FILLING A BOX------------------
        while(i_model<num_parts)  { //persist with each model until success or impossible
            go_on=true;
            //attempt to update inventory.  if not successful, keep using current memory of inventory
            if(binInventory.update()) {
                binInventory.get_inventory(current_inventory);
            }
            current_model = desired_models_wrt_world[i_model];
            //build "part" description for destination
            shipmentFiller.model_to_part(current_model,place_part,inventory_msgs::Part::QUALITY_SENSOR_1);
            std::string part_name(place_part.name);
            ROS_INFO_STREAM("attempting a placement of "<<place_part<<endl);

            //try, try again to put this part in the  box
            if (!shipmentFiller.get_part_and_place_in_box(current_inventory,place_part)) {
                  //for some reason, it is not  possible to place this  part, regardless of quality and placement precision
                  //move along to the next part  in the shipment
                  ROS_WARN("unsuccessful with pick/place of model %d",i_model);
                  ROS_WARN("moving on to try the next part in the shipment");
                  go_on=false; //give up on placing this model in the box
                  i_model++; //give up on placing this part; it seems to be impossible
            }
            
            
                        
            if (go_on) {
                ROS_INFO("successful part placement; should inspect before release");
                go_on = shipmentFiller.replace_faulty_parts_inspec1(shipment);
            }
            
            
            if (go_on) {
                ROS_INFO("attempting part release; enter 1: ");
                cin>>ans;
                go_on = robotBehaviorInterface.release_and_retract(); //release the part
            }
            
            if (go_on) {
                ROS_INFO("attempting to  regrasp part from box; should use camera to get pose...  ");
                //pick_part_from_box(Part part, double timeout)
                go_on = robotBehaviorInterface.pick_part_from_box(place_part);
            }
            
            if (go_on) {
                //adjust part locations in box: 
                go_on = shipmentFiller.adjust_shipment_part_locations(desired_models_wrt_world);
            }
            

            if (go_on) { //if here, 
                ROS_INFO("declaring success, and moving on to the  next product");
              i_model++;
            }

        }
        ROS_INFO("done processing shipment; advancing box");
        ROS_WARN("SHOULD HAVE A  LOOP HERE  TO PROCESS MORE SHIPMENTS");
                 
            //if making conveyor seperate action server, can have inf loop above this
            //advance shipment to next inspection  station:
            advanced_shipment_on_conveyor= 
                shipmentFiller.advance_shipment_on_conveyor(BOX_INSPECTION2_LOCATION_CODE);
            //fix any faulty parts at inspection station Q2
            //dummy fnc for now
            replaced_faulty_parts = shipmentFiller.replace_faulty_parts_inspec2(shipment);

            //report shipment has been filled:
            shipmentFiller.current_shipment_has_been_filled();
            //advance order to drone dock:
            advanced_shipment_on_conveyor= 
                shipmentFiller.advance_shipment_on_conveyor(DRONE_DOCK_LOCATION_CODE);
            //send notice to drone:
            
            reported_shipment_to_drone= shipmentFiller.report_shipment_to_drone();
            //send robot to waiting pose; update inventory
        }    
            ROS_WARN("stopping after single shipment...FIX ME!");
            return 0;
}

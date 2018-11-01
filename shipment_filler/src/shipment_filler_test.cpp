//This node develops on simple_shipment_filler by trying to use the optimizer node


#include<order_manager/order_manager.h>
#include<shipment_filler/ShipmentFiller.h>
#include <robot_behavior_interface/RobotBehaviorInterface.h>
#include<bin_inventory/bin_inventory.h>
#include<box_inspector/box_inspector.h>
#include<conveyor_as/ConveyorInterface.h>

const double COMPETITION_TIMEOUT=500.0; 

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
    ROS_INFO("instantiating a ConveyorInterface");
    ConveyorInterface conveyorInterface(&nh);    

    inventory_msgs::Part pick_part,place_part, observed_part;
    inventory_msgs::Inventory current_inventory;
    osrf_gear::Shipment shipment;
    osrf_gear::Order order;
    geometry_msgs::PoseStamped box_pose_wrt_world;  
    vector<osrf_gear::Model>  desired_models_wrt_world,missing_parts;
    osrf_gear::Model current_model;
    //inventory_msgs::Part current_part;
    int bin_num,partnum;
    geometry_msgs::PoseStamped part_pose;
    int a;//remove after debyg
    bool got_shipment = false;
    bool successfully_filled_order = false;
    bool init_pack_shipment= false;
    bool replaced_faulty_parts=false;
    bool adjusted_part_locations=false;
    //bool corrected_dropped_part=false;
    bool reported_shipment_to_drone=false;
    bool advanced_shipment_on_conveyor=false;
    bool go_on= true;
    bool order_fulfilled=false;
    bool checked_for_order_update=false;
    bool order_updated;
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
    double current_time;    
        
    //MAKE BOX CONTROL AN ACTION SERVER for multi-tasking
    while(ros::ok()) { //check for conditions where we might have to stop otherwise
    	ROS_INFO("starting round");
    	ROS_INFO("getting a box into position: ");
    	/* THE FOLLOWING IS BLOCKING; REPLACE IT */
    	//advanced_shipment_on_conveyor= 
     	//           shipmentFiller.advance_shipment_on_conveyor(BOX_INSPECTION1_LOCATION_CODE);
    
    	//FIND HOW TO INSERT THIS INSTEAD
    	//the following starts the conveyor action server moving the first box into position at inspection station Q1
    	conveyorInterface.move_new_box_to_Q1();
    	// the above is not a blocking function.  Can fetch a part, but don't try to deposit it in box until
    	// box is in position



    	//first box is at Q1 inspection station; get a shipment:
    	//got_shipment = orderManager.choose_shipment(shipment);
    
    	//can't go any further until receive new shipment request, or until near competition expiration
        //ROS_INFO("ROSISOK!");
       
        //now have a shipment; start  processing it
        successfully_filled_order = false;    
        while(!successfully_filled_order) {
        	while (!orderManager.choose_shipment(shipment)) {
          		ROS_INFO("waiting for shipment");
          		ros::Duration(0.5).sleep();
				ros::spinOnce();

        	}	
        	current_time=ros::Time::now().toSec();
        	if(current_time - competition_start_time > 460 ) {
            	ROS_WARN("TIMES UP!");
            	break;
        	}
        
   
            ROS_INFO_STREAM("shipment to be filled: " << shipment << endl);
            //prep for drone request: set shipment name
            if(binInventory.update()) {
                    binInventory.get_inventory(current_inventory);
            }

            
            boxInspector.compute_shipment_poses_wrt_world(shipment, boxInspector.NOM_BOX1_POSE_WRT_WORLD, desired_models_wrt_world);
        	ROS_INFO("REPLACE WITH RESULT MESSAGE FROM ACTION SERVER. For now, press 1");
                       
    		int num_parts = desired_models_wrt_world.size();
    		ROS_INFO("trying to fill box with %d products",num_parts);
                int i_model=0;  //SINCE WE PLACED THE FIRST PART ALREADY? START WITH 1?
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

                if (!shipmentFiller.get_part_and_prepare_place_in_box(current_inventory,place_part)) {
                  //for some reason, it is not  possible to place this  part, regardless of quality and placement precision
                  //move along to the next part  in the shipment
                	ROS_WARN("unsuccessful with pick/place of model %d",i_model);
                	ROS_WARN("moving on to try the next part in the shipment");
                    go_on=false; //give up on placing this model in the box
                    i_model++; //give up on placing this part; it seems to be impossible
                }
            
                if (go_on) {
                	ROS_INFO("observe pose of grasped part in approach pose");
                    ros::Duration(1.0).sleep(); //let robot stabilize;
                    ROS_WARN("NEED TO GET OBSERVED PART POSE HERE...");
                    ROS_WARN("result should be called observed_part");
                    //WRITE THIS FNC: given that we are grasping place_part in approach pose above box, 
                // get actual pose of this part from the box camera
                // DO watch out for timeout; if timeout, just proceed with blind placement
                   	
                   	while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_ESTIMATED_AT_Q1 &&
                        (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SEEN_AT_Q1)) {
                    ros::spinOnce();
                    ros::Duration(0.1).sleep();
                    ROS_INFO("waiting for conveyor to advance a box to Q1...");
                }
                

                    boxInspector.get_observed_part_pose(place_part, observed_part);
                	
                	if (boxInspector.get_box_pose_wrt_world(box_pose_wrt_world)) {
                    //update desired part poses w/rt world:
                    boxInspector.compute_shipment_poses_wrt_world(shipment, box_pose_wrt_world, desired_models_wrt_world);
                	}

                //when above fnc call is working, call the following
                    ROS_WARN("should now call re_evaluate_approach_and_place_poses...or put in shipmentFiller fnc");               
                    if(!robotBehaviorInterface.re_evaluate_approach_and_place_poses(observed_part,place_part)) {
                    //can't reach corrected poses.  May as well just drop the part
                    	ROS_WARN("dropping part");
                        robotBehaviorInterface.discard_grasped_part(place_part);
                        go_on=false;
                    }
                }
            //if successful to here, use the newly computed poses:
				if(go_on) {
                    ROS_INFO("using newly adjusted approach and place poses to place part in box, no release");                        
                    go_on = robotBehaviorInterface.place_part_in_box_from_approach_no_release(place_part);
                }
                        
                if (go_on) {
                 	ROS_INFO("successful part placement; should inspect before release");
                    go_on = shipmentFiller.replace_faulty_parts_inspec1(shipment);
                }
           
                if (go_on) {
                	ROS_INFO("attempting part release; enter 1: ");
                    go_on = robotBehaviorInterface.release_and_retract(); //release the part
                }

                if (go_on) { //if here, 
                    ROS_INFO("declaring success, and moving on to the  next product");
                    i_model++;
                }
              /*  ROS_INFO("checking for unwanted parts");
            if(!shipmentFiller.remove_unwanted_parts(desired_models_wrt_world)) {
                ROS_INFO("either no unwanted parts or couldnt remove");
            }*/

                    //THIS IS WHERE ORDER UPDATE MUST BE CHECKED
            }
            /*if(go_on) {
                if(!shipmentFiller.adjust_shipment_part_locations(desired_models_wrt_world)) {
                    ROS_INFO("Unable to post adjust parts");
            	}
            }*/
            ROS_INFO("Are you sure bro?");
            successfully_filled_order = true; 
        }

        orderManager.current_shipment_has_been_filled();
        advanced_shipment_on_conveyor= 
                shipmentFiller.advance_shipment_on_conveyor(DRONE_DOCK_LOCATION_CODE);
        shipmentFiller.set_drone_shipment_name(shipment);

        reported_shipment_to_drone= shipmentFiller.report_shipment_to_drone();
        if(binInventory.update()) {
                binInventory.get_inventory(current_inventory);
        }


    }
}            

//modified to (partially) test revised dropoff:
//goto computed approach pose, get camera measurement of part pose, recomputed approach and dropoff poses,
//do dropoff from approach (no release)

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
#include<conveyor_as/ConveyorInterface.h>
#include<optimizer_func/optimizer_msgs.h>
#include <kuka_move_as/GripperInterface.h>
#include <queue>

const double COMPETITION_TIMEOUT = 500.0; // need to  know what this is for the finals;
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
    std_srvs::Trigger srv; // Combination of the "request" and the "response".
    start_client.call(srv); // Call the start Service.
    if (!srv.response.success) { // If not successful, print out why.
        ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
    } else {
        ROS_INFO("Competition started!");
    }
}

bool done_with_this_shipment(vector<bool> parts_checklist) {
    int nparts = parts_checklist.size();
    bool am_done = true;
    for (int i = 0; i < nparts; i++) {
        am_done = am_done && parts_checklist[i];
    }
    return am_done;
}

//somewhat odd fnc.  have a parts_checklist of booleans, for which a part that has been treated is assigned "true"
//also have a vector of indices of parts that are misplaced.  These indices refer to the original desired list of parts in the order
//some of these noted, misplaced parts may have been treated (attempt at repositioning).  If so, the corresponding 
//parts_checklist[desired_part_index] will be true.  Therefore, this misplaced part ID should not be treated again
//search for some part ID that is on the misplaced list and is also not yet treated
//if found, return true, and set reference args for the element of misplaced list and the  corresponding desired  part index
//note: the latter is easy enough to find as desired_part_index = part_indices_misplaced[misplaced_part_index], so returning it is
//not really necessary
bool get_part_index_to_reposition(vector<int> part_indices_misplaced, vector<bool> parts_checklist, int &misplaced_part_index, int &desired_part_index) {
    //run through the indices of misplaced parts,  and find first occurrence of untreated part
    int num_misplaced = part_indices_misplaced.size();
    for (misplaced_part_index=0;misplaced_part_index<num_misplaced;misplaced_part_index++) {
        desired_part_index = part_indices_misplaced[misplaced_part_index];
        if (!parts_checklist[desired_part_index]) {
            //found an occurrence of part on misplaced list that has NOT been previously treated
            return true;  
        }
    }
    //did not find match, so there are no untreated misplaced parts
    return false;
}

bool get_part_index_to_acquire(vector<int> part_indices_missing,vector<bool> parts_checklist, int &missing_part_index,int &desired_part_index) {
    int num_missing = part_indices_missing.size();
    for (missing_part_index=0;missing_part_index<num_missing;missing_part_index++) {
        desired_part_index = part_indices_missing[missing_part_index];
        if (!parts_checklist[desired_part_index]) {
            //found an occurrence of part on misplaced list that has NOT been previously treated
            return true;  
        }
    }
    //did not find match, so there are no untreated missing parts
    return false;    
}

int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "shipment_filler"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    //here are a bunch of useful objects.  Most of these are also instantiated  "ShipmentFiller"
    //still deciding what and how much to expose at this top level
    int ans;
    inventory_msgs::Part bad_part;
    std::vector<bool> parts_checklist; //, done_with_part_Q2;
    string current_shipment_name("no_active_shipment");
    string no_shipment_name("no_active_shipment");

    //ROS_INFO("main: instantiating an object of type OrderManager");
    //OrderManager orderManager(&nh); //shipmentFiller also owns one of these, which is public
    ROS_INFO("instantiating a RobotBehaviorInterface");
    RobotBehaviorInterface robotBehaviorInterface(&nh); //shipmentFiller owns one as well
    ROS_INFO("instantiationg a BinInventory object");
    BinInventory binInventory(&nh); //shipmentFiller owns one of these, which is public
    ROS_INFO("instantiating a BoxInspector");
    BoxInspector boxInspector(&nh);
    ROS_INFO("instantiating a ShipmentFiller");
    ShipmentFiller shipmentFiller(&nh);
    ROS_INFO("instantiating a ConveyorInterface");
    ConveyorInterface conveyorInterface(&nh);
    ROS_INFO("instantiating a GripperInterface");    
    GripperInterface gripperInterface(nh);
    
    std::queue <osrf_gear::Shipment> drone_shipment_queue;
    
    
    optimizer_func::optimizer_msgs optimizer_msg;

    ros::ServiceClient optimizer_client = nh.serviceClient<optimizer_func::optimizer_msgs>("optimizer");
    while (!optimizer_client.exists()) {
        ROS_INFO("waiting for optimization service...");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to optimization service");

    osrf_gear::Shipment shipment_loaded, shipment_orphaned, shipment_missing, shipment_reposition, empty_shipment;
    osrf_gear::Shipment empty_shipment_loaded, empty_shipment_orphaned, empty_shipment_missing, empty_shipment_reposition;

    inventory_msgs::Part pick_part, place_part, observed_part, remove_part;
    inventory_msgs::Inventory current_inventory;
    osrf_gear::Shipment shipment, Q1_shipment, Q2_shipment,drone_shipment; //keep track of shipments at Q1 and and Q2
    osrf_gear::Order order;
    geometry_msgs::PoseStamped box_pose_wrt_world;
    vector<osrf_gear::Model> desired_models_wrt_world;
    osrf_gear::Model current_model;

    //bool BoxInspector::update_inspection(vector<osrf_gear::Model> desired_models_wrt_world,
    vector<osrf_gear::Model> satisfied_models_wrt_world;
    vector<osrf_gear::Model> misplaced_models_actual_coords_wrt_world;
    vector<osrf_gear::Model> misplaced_models_desired_coords_wrt_world;
    vector<osrf_gear::Model> missing_models_wrt_world;
    vector<osrf_gear::Model> orphan_models_wrt_world;
    vector<int> part_indices_missing;
    vector<int> part_indices_misplaced;
    vector<int> part_indices_precisely_placed;
    //vector<int> problem_model_acquisition_indices;
    //vector<int> problem_model_reposition_indices;  
    vector<int> problem_model_indices;

    //inventory_msgs::Part current_part;
    int bin_num, partnum;
    int n_precise, n_imprecise, n_missing, n_orphaned;
    geometry_msgs::PoseStamped part_pose;
    int debug; //remove after debug
    bool got_shipment = false;
    bool successfully_filled_order = false;
    bool init_pack_shipment = false;
    bool replaced_faulty_parts = false;
    bool adjusted_part_locations = false;
    //bool corrected_dropped_part=false;
    bool reported_shipment_to_drone = false;
    bool advanced_shipment_on_conveyor = false;
    bool go_on = true;
    bool waiting_on_drone=false;

    ROS_INFO("attempting to get an inventory update; pretty much screwed until this is possible");
    while (!binInventory.update()) {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }

    ROS_INFO("got initial inventory");
    binInventory.get_inventory(current_inventory);
    conveyorInterface.move_new_box_to_Q1();

    empty_shipment.shipment_type = no_shipment_name.c_str(); //has a name, but empty Product vector

    //initialize first request to optimizer: empty shipment

    //populate a request for the shipment  optimizer service:
    shipmentFiller.modelvec_to_shipment(no_shipment_name, satisfied_models_wrt_world, boxInspector.NOM_BOX1_POSE_WRT_WORLD, empty_shipment_loaded);
    shipmentFiller.modelvec_to_shipment(no_shipment_name, orphan_models_wrt_world, boxInspector.NOM_BOX1_POSE_WRT_WORLD, empty_shipment_orphaned);
    shipmentFiller.modelvec_to_shipment(no_shipment_name, missing_models_wrt_world, boxInspector.NOM_BOX1_POSE_WRT_WORLD, empty_shipment_missing);
    shipmentFiller.modelvec_to_shipment(no_shipment_name, misplaced_models_actual_coords_wrt_world, boxInspector.NOM_BOX1_POSE_WRT_WORLD, empty_shipment_reposition);
    optimizer_msg.request.loaded = empty_shipment_loaded;
    optimizer_msg.request.orphaned = empty_shipment_missing;
    optimizer_msg.request.reposition = empty_shipment_reposition;
    optimizer_msg.request.giving_up = optimizer_func::optimizer_msgsRequest::NOT_GIVING_UP;
    optimizer_msg.request.inspection_site = optimizer_func::optimizer_msgsRequest::Q1_STATION;

    //MAKE BOX CONTROL AN ACTION SERVER for multi-tasking
    ROS_INFO("initiating action server to get first box into position: ");

    ROS_INFO("starting the competition");
    start_competition(nh); //start the competition

    double competition_start_time = ros::Time::now().toSec(); //check the clock= start time

    //START OF LONG PREAMBLE:
    //conveyor action server is moving first box to Q1
    //suspend, waiting for first order
    // attempt to acquire first (or subsequent) part of order from inventory and hold it in view  of box1 cam
    //wait for box to show up at Q1
    //get updated box pose
    //recompute destination  poses w/rt world
    //get grasp transform from camera
    //recompute key poses for placement
    //place part, release and retract
    //do box inspection in prep for next call to shipment optimizer

    //suspend here until receive first order; (box is still moving to Q1 destination)

    string Q1_shipment_name = no_shipment_name;
    string Q2_shipment_name = no_shipment_name;
    string drone_shipment_name = no_shipment_name;

    string prior_Q1_shipment_name = no_shipment_name;

    bool have_active_shipment_Q1 = false;
    bool have_active_shipment_Q2 = false;
    while (Q1_shipment_name == no_shipment_name) {
        ROS_INFO("waiting for first order...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        optimizer_client.call(optimizer_msg);
        shipment = optimizer_msg.response.shipment;
        //string rtn_name(shipment.shipment_type);
        Q1_shipment_name = std::string(shipment.shipment_type); //rtn_name;
        ROS_INFO_STREAM("optimizer returned shipment name: " << Q1_shipment_name << endl);
    }
    ROS_INFO("Got first shipment: ");
    have_active_shipment_Q1 = true;
    prior_Q1_shipment_name = Q1_shipment_name;
    Q1_shipment = shipment; //make a copy of this shipment here, noting it is associated with station Q1
    ROS_INFO_STREAM("initial shipment: " << endl << Q1_shipment << endl);

    ROS_INFO("computing estimated destination poses based on expected box pose");
    boxInspector.compute_shipment_poses_wrt_world(Q1_shipment, boxInspector.NOM_BOX1_POSE_WRT_WORLD, desired_models_wrt_world);
    successfully_filled_order = false; //obsolete

    //acquire first part while waiting on box; move this part to camera inspection above box
    int num_models_in_order = desired_models_wrt_world.size();
    parts_checklist.resize(num_models_in_order);
    //clear the parts checklist--we have not yet satisfied nor even attempted addressing any of these desired parts
    for (int i = 0; i < num_models_in_order; i++) parts_checklist[i] = false;
    int i_model = 0;
    current_model = desired_models_wrt_world[i_model]; //try to get the first model of this shipment ready
    //build "part" description for destination
    shipmentFiller.model_to_part(current_model, place_part, inventory_msgs::Part::QUALITY_SENSOR_1);
    
    string place_part_name(place_part.name);
    int partnum_in_inventory;
    bool part_in_inventory = binInventory.find_part(current_inventory, place_part_name, pick_part, partnum_in_inventory);
    
    ROS_INFO_STREAM("initial chosen pick_part: " << endl << pick_part << endl);
    ROS_INFO_STREAM("initial chosen place_part: " << endl << place_part << endl);
    
    //evaluate_key_pick_and_place_poses
    //    bool evaluate_key_pick_and_place_poses(Part sourcePart, Part destinationPart, double timeout = MAX_ACTION_SERVER_WAIT_TIME);
    go_on = robotBehaviorInterface.evaluate_key_pick_and_place_poses(pick_part,place_part);
    if (!go_on) {
        ROS_WARN("could not compute key pickup and place poses for this part source and destination");
    }
    
    if (go_on) { 
        ROS_INFO_STREAM("attempting to acquire and place part"); 
        go_on = shipmentFiller.get_part_and_prepare_place_in_box(current_inventory, place_part); 
    }

    //have to wait for box to show up at Q1 before can attempt placement
    int nprint = 0;
    while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_ESTIMATED_AT_Q1 &&
            (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SEEN_AT_Q1)) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        nprint++;
        if (nprint % 10 == 0) {
            ROS_INFO("waiting for conveyor to advance a box to Q1...");
        }
    }

    //update box pose,  if possible              
    if (boxInspector.get_box_pose_wrt_world(box_pose_wrt_world)) {
        ROS_INFO_STREAM("box seen at: " << box_pose_wrt_world << endl);
        //update desired part poses w/rt world:
        boxInspector.compute_shipment_poses_wrt_world(shipment, box_pose_wrt_world, desired_models_wrt_world);
        //improve on place_part, using known box coords:
        current_model = desired_models_wrt_world[i_model]; //try to get the first model of this shipment ready
        //rebuild "part" description for destination
        shipmentFiller.model_to_part(current_model, place_part, inventory_msgs::Part::QUALITY_SENSOR_1);
        //re-evaluate pick/place poses given refined destination coords:
        if (!robotBehaviorInterface.evaluate_key_pick_and_place_poses(pick_part,place_part)) {
            ROS_WARN("recomputation of dropoff plan using updated box coords was invalid; Odd; Code  err?");
            go_on = false;
        }
    }    
        

    if (!gripperInterface.isGripperAttached()) { 
        ROS_WARN("part not grasped!");
        go_on=false;
        ROS_WARN("return to cruise pose--finish me");
    }
    
    //see if part is viewable:
    bool did_observe_grasped_part=false;
    if (go_on) {
        //get grasp transform from camera
      //recompute key poses for placement
 
      ROS_WARN("conveyor halted; get pose of grasped  part");
        //bool BoxInspector::get_grasped_part_pose_wrt_world(inventory_msgs::Part &observed_part) {
      observed_part = place_part; //need the part name for observation
      did_observe_grasped_part = boxInspector.get_grasped_part_pose_wrt_world(observed_part); //updates observed_part
      if (did_observe_grasped_part) {
         ROS_INFO_STREAM("observed grasped part: " << endl << observed_part << endl);
         ROS_WARN("calling re_evaluate_approach_and_place_poses");
         if (!robotBehaviorInterface.re_evaluate_approach_and_place_poses(observed_part, place_part)) {
             //go back to original key poses:
             robotBehaviorInterface.evaluate_key_pick_and_place_poses(pick_part,place_part);
             ROS_WARN("re-evaluation of dropoff poses was not successful;  using open-loop plan");
         }         
      }
      else { 
          ROS_WARN("could not observe grasped part; using open-loop dropoff plan");
          //retain original  key poses
      }
    }

    if (!go_on) {
        if (gripperInterface.isGripperAttached()) {
            ROS_WARN("giving up on this part; retract and discard");
            ROS_WARN("FIX ME");
            //robotBehaviorInterface.discard_grasped_part(place_part);
            robotBehaviorInterface.release_and_retract();
        }
    }

    if (go_on) {
        ROS_INFO("using best estimate approach and place poses to place part in box, no release");
        ROS_INFO_STREAM("place_part: "<<place_part<<endl);
        go_on = robotBehaviorInterface.place_part_in_box_from_approach_no_release(place_part);
    }
      
    if (go_on) {
         //check if this part passes quality inspection:
         if (shipmentFiller.get_bad_part_Q1(bad_part)) {
            ROS_WARN("saw bad part; discard it");
            //do discard here
            robotBehaviorInterface.discard_grasped_part(bad_part);
      } else {
        //release part and retract arm
        ROS_INFO("quality sensor did not declare part as bad; leave it");
        robotBehaviorInterface.release_and_retract();
      }
    }


    //END OF LONG PREAMBLE

    while (ros::ok()) {
        ros::spinOnce();        
        if (!conveyorInterface.drone_depot_sees_box()) waiting_on_drone=false; //sprinkle these throughout
        
        if (!have_active_shipment_Q2 && !have_active_shipment_Q1) { //populate service msg w/ empty shipment
            ROS_WARN("no active orders; will request order from optimizer");
            //define current Q1 shipment as an empty order
            //need a new shipment; load up request with empty shipment, station Q1
            optimizer_msg.request.loaded = empty_shipment_loaded;
            optimizer_msg.request.orphaned = empty_shipment_missing;
            optimizer_msg.request.reposition = empty_shipment_reposition;
            optimizer_msg.request.giving_up = optimizer_func::optimizer_msgsRequest::NOT_GIVING_UP;
            optimizer_msg.request.inspection_site = optimizer_func::optimizer_msgsRequest::Q1_STATION;
        }

        if (have_active_shipment_Q1) { //populate the service message based on inspection:
            ROS_INFO("working on active shipment at Q1");
            //make sure box is present:
            nprint = 0;
            while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_ESTIMATED_AT_Q1 &&
                    (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SEEN_AT_Q1)) {
                ros::spinOnce();
                if (!conveyorInterface.drone_depot_sees_box()) waiting_on_drone=false; //sprinkle these throughout
                ros::Duration(0.1).sleep();
                nprint++;
                if (nprint % 10 == 0) {
                    ROS_INFO("waiting for conveyor to advance a box to Q1...");
                }
            }
            ROS_INFO("box is in position");
            //do a box inspection to see how we are  doing:
            if (boxInspector.get_box_pose_wrt_world(box_pose_wrt_world)) {
                //update desired part poses w/rt world:
                boxInspector.compute_shipment_poses_wrt_world(shipment, box_pose_wrt_world, desired_models_wrt_world);
            }
            //inspect the box: 
            //xxx this will get updated with part-identity vectors and bad parts will be orphaned parts
            boxInspector.update_inspection(desired_models_wrt_world, satisfied_models_wrt_world,
                    misplaced_models_actual_coords_wrt_world, misplaced_models_desired_coords_wrt_world,
                    missing_models_wrt_world, orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,part_indices_precisely_placed);
            n_precise = satisfied_models_wrt_world.size();
            n_imprecise = misplaced_models_actual_coords_wrt_world.size();
            n_missing = missing_models_wrt_world.size();
            n_orphaned = orphan_models_wrt_world.size();
            ROS_WARN("after box inspection: ");
            ROS_INFO("n_precise =  %d; n_imprecise = %d; n_missing = %d; n_orphaned = %d", n_precise,
                    n_imprecise, n_missing, n_orphaned);

            ROS_INFO("precisely-placed parts checklist: ");
            for (int i=0;i<n_precise;i++) {
                
                int good_part_index = part_indices_precisely_placed[i];
                ROS_INFO("watching for segfaults. Good part index: %d press 1", good_part_index);
                //cin>>debug;
                parts_checklist[good_part_index] = true;  //mark these parts as "done"     
                ROS_INFO("part %d is precise",good_part_index);
            }
            //populate a service message for optimizer:    
            shipmentFiller.modelvec_to_shipment(Q1_shipment_name, satisfied_models_wrt_world, box_pose_wrt_world, shipment_loaded);
            shipmentFiller.modelvec_to_shipment(Q1_shipment_name, orphan_models_wrt_world, box_pose_wrt_world, shipment_orphaned);
            shipmentFiller.modelvec_to_shipment(Q1_shipment_name, missing_models_wrt_world, box_pose_wrt_world, shipment_missing);
            shipmentFiller.modelvec_to_shipment(Q1_shipment_name, misplaced_models_actual_coords_wrt_world, box_pose_wrt_world, shipment_reposition);
            optimizer_msg.request.loaded = shipment_loaded;
            optimizer_msg.request.orphaned = shipment_orphaned;
            optimizer_msg.request.reposition = shipment_reposition;
            optimizer_msg.request.giving_up = optimizer_func::optimizer_msgsRequest::NOT_GIVING_UP;
            optimizer_msg.request.inspection_site = optimizer_func::optimizer_msgsRequest::Q1_STATION;

            //check if shipment-filler is giving up, or advising "done"
            //keep this next line: know when to say when--including "success" with all parts correctly placed
            ROS_INFO("inspecting checklist: ");
            if (done_with_this_shipment(parts_checklist)) {
                ROS_INFO("checklist complete!  recommend moving along/giving up");
                optimizer_msg.request.giving_up = optimizer_func::optimizer_msgsRequest::GIVING_UP;
            }
            else {
                ROS_INFO("checklist not complete");
            }
        }
        //update call to optimizer from  Q1, assuming don't have an active Q2 shipment
        if (!have_active_shipment_Q2) {
            //TRY TO UPDATE THE SHIPMENT HERE FOR Q1
            ROS_INFO("requesting a shipment update");
            optimizer_client.call(optimizer_msg); //update the shipment request from Q1, possibly with advice to give-up/move-along

            //check first if advancing box to Q2:
            if (optimizer_msg.response.decision == optimizer_func::optimizer_msgsResponse::ADVANCE_THIS_BOX_TO_Q2) {
                ROS_INFO("recommended moving box from Q1 to  Q2");
                Q2_shipment = Q1_shipment; //same shipment specifications,  but now at Q2
                Q1_shipment = empty_shipment;
                Q1_shipment_name = no_shipment_name;
                prior_Q1_shipment_name = Q1_shipment_name;
                Q2_shipment_name = Q1_shipment_name; //do NOT permit renaming this shipment

                have_active_shipment_Q1 = false;
                have_active_shipment_Q2 = true;

                conveyorInterface.move_box_Q1_to_Q2();
            }
        }
        ros::spinOnce();        
        if (!conveyorInterface.drone_depot_sees_box()) waiting_on_drone=false; //sprinkle these throughout
        //see if have any work to do at Q1, assuming no Q2
        //only case should be USE_CURRENT_BOX
        if (!have_active_shipment_Q2) {
            //already did box-inspection update and order request
            //but if order has changed, need to update before doing manipulations
            //note--order may change from empty to real, or from low-priority to high-priority with changes,  e.g.
            shipment = optimizer_msg.response.shipment;
            Q1_shipment = shipment; //synonym
            Q1_shipment_name = std::string(shipment.shipment_type); //shipment name
            ROS_INFO_STREAM("optimizer returned shipment name: " << Q1_shipment_name << endl);

            //need to update inspection if shipment has changed in process:
            if (Q1_shipment_name != prior_Q1_shipment_name) { //this is the only way to activate have_active_shipment_Q1
                ROS_INFO_STREAM("new shipment name received: " << Q1_shipment_name << endl);
                prior_Q1_shipment_name = Q1_shipment_name;
                have_active_shipment_Q1 = true;
                num_models_in_order = shipment.products.size();
                //use parts_checklist to keep track of parts that are satisfied, or parts we are giving  up on
                parts_checklist.resize(num_models_in_order);
                for (int i = 0; i < num_models_in_order; i++) parts_checklist[i] = false;
                //have to do a new assessment before handling parts:

                //since this is a new order, compute new desired  part poses
                boxInspector.get_box_pose_wrt_world(box_pose_wrt_world);
                boxInspector.compute_shipment_poses_wrt_world(shipment, box_pose_wrt_world, desired_models_wrt_world);
                //do a box inspection to compare current box contents to new order
                boxInspector.update_inspection(desired_models_wrt_world, satisfied_models_wrt_world,
                    misplaced_models_actual_coords_wrt_world, misplaced_models_desired_coords_wrt_world,
                    missing_models_wrt_world, orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,part_indices_precisely_placed);
                n_precise = satisfied_models_wrt_world.size();
                n_imprecise = misplaced_models_actual_coords_wrt_world.size();
                n_missing = missing_models_wrt_world.size();
                n_orphaned = orphan_models_wrt_world.size();
                ROS_INFO("after inspection: n_precise =  %d; n_imprecise = %d; n_missing = %d; n_orphaned = %d", n_precise,
                        n_imprecise, n_missing, n_orphaned);

            }
        }
        ros::spinOnce();        
        if (!conveyorInterface.drone_depot_sees_box()) waiting_on_drone=false; //sprinkle these throughout
        //eventually, optimizer will give us an active order at Q1
        if (have_active_shipment_Q1) {
            int missing_part_index,misplaced_part_index,desired_part_id;
        
            ROS_INFO("processing shipment at Q1");

            //at this point,  have_active_shipment_Q1, and order and box inspection have been updated; get to work on one part

            //shipment response has two possibilities: USE_CURRENT_BOX or ADVANCE_THIS_BOX_TO_Q2
            //already handled ADVANCE_THIS_BOX_TO_Q2, so expect USE_CURRENT_BOX

            //check if current box should continue to be used:
            //this is the most interesting case--calls for part manipulations
            if (optimizer_msg.response.decision == optimizer_func::optimizer_msgsResponse::USE_CURRENT_BOX) {
                ROS_INFO("optimizer says to use this box");
                //do actions in this order: 1) unload orphaned parts; 2) adjust imprecisely placed parts; 3) load new parts
                //note: advised shipment might be NEW:
                //Q1_shipment = optimizer_msg.response.shipment;
                //update inspection:
                boxInspector.update_inspection(desired_models_wrt_world, satisfied_models_wrt_world,
                    misplaced_models_actual_coords_wrt_world, misplaced_models_desired_coords_wrt_world,
                    missing_models_wrt_world, orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,part_indices_precisely_placed);
                n_precise = satisfied_models_wrt_world.size();
                n_imprecise = misplaced_models_actual_coords_wrt_world.size();
                n_missing = missing_models_wrt_world.size();
                n_orphaned = orphan_models_wrt_world.size();
                ROS_INFO("after inspection: n_precise =  %d; n_imprecise = %d; n_missing = %d; n_orphaned = %d", n_precise,
                        n_imprecise, n_missing, n_orphaned);
                //FAKE...
                /*
                if ((orphan_models_wrt_world.size() == 0) && (missing_models_wrt_world.size() == 0)) {
                    //FAKE: call this done:
                    for (int i = 0; i < num_models_in_order; i++) parts_checklist[i] = true;
                    ROS_WARN("fake for debug: no orphans and total parts in box = order size; recommmending move along");
                }
                */
                //highest priority: remove an orphaned part
                if (orphan_models_wrt_world.size() > 0) {
                    ROS_INFO("removing an orphaned part");
                    //remove orphaned parts:  FIX ME! watch out for infinite loop
                    current_model = orphan_models_wrt_world[0];
                    shipmentFiller.model_to_part(current_model, remove_part, inventory_msgs::Part::QUALITY_SENSOR_1);
                    robotBehaviorInterface.pick_part_from_box(remove_part);
                    robotBehaviorInterface.discard_grasped_part(remove_part);
                }//second priority: adjust a part position
                
                //bool get_part_index_to_reposition(vector<int> part_indices_misplaced, vector<bool> parts_checklist, int &part_index) {
                //            int misplaced_part_index, desired_part_id;
                //here is block of code to operate on misplaced  parts;
                // only do so if have a misplaced  part for which repositioning was not already attempted
                else if (get_part_index_to_reposition(part_indices_misplaced,parts_checklist,misplaced_part_index,desired_part_id) ) {
                                     //xxx FAKE: dont attempt to adjust misplaced parts:
                    ROS_INFO("found part index %d  of desired shipment is misplaced and not yet attempted to reposition",desired_part_id);
                            ROS_WARN("should do part relocation here; merely mark the part to be ignored via done_with_part_Q1");
                            parts_checklist[desired_part_id] = true; //give up without trying
                            /**/

                            current_model = misplaced_models_actual_coords_wrt_world[misplaced_part_index];
                            //int model_index = part_indices_misplaced[0];
                            shipmentFiller.model_to_part(current_model, pick_part, inventory_msgs::Part::QUALITY_SENSOR_1);
                            //and desired part pose:
                            current_model = misplaced_models_desired_coords_wrt_world[misplaced_part_index];
                            shipmentFiller.model_to_part(current_model, place_part, inventory_msgs::Part::QUALITY_SENSOR_1);
                            ROS_INFO_STREAM("should move part ID "<<desired_part_id<<" from "<<endl<<pick_part<<endl<<"to:"<<endl<<place_part<<endl);
                            ROS_WARN("giving up without trying");
                            //robotBehaviorInterface.adjust_part_location_with_release(pick_part,place_part); 
                            //regardless, mark this part as done:
                            parts_checklist[desired_part_id] = true; 
                }      

                //MAIN MANIPULATION CONDITION:
                //third priority: get a new part and try to place in box; 
                //bool get_part_index_to_acquire(part_indices_missing,parts_checklist,missing_part_index,desired_part_id) {
                //bool get_part_index_to_acquire(vector<int> part_indices_missing,vector<bool> parts_checklist, int &missing_part_index,int &desired_part_index) {
                
                else if (get_part_index_to_acquire(part_indices_missing,parts_checklist,missing_part_index,desired_part_id) ) 
                {   //get a viable index for a missing part
                    ROS_INFO("trying to acquire a missing part");
                    ros::spinOnce();
                    if(binInventory.update()) {
                    binInventory.get_inventory(current_inventory); //update the inventory
                    } 
                    //have to use the old values in current_inventory otherwise. During sensor blackout, get_inventory returns values seen before the blackout, causing robot to try parts that dont exist
                    //do the sequence to pick and place:

                    //don't bother to recompute desired part poses w/rt world--assume box has not moved since last time
                    current_model = missing_models_wrt_world[missing_part_index];
                    ROS_INFO_STREAM("working on this model: " << current_model << endl);
                    shipmentFiller.model_to_part(current_model, place_part, inventory_msgs::Part::QUALITY_SENSOR_1);
                    ROS_INFO_STREAM("corresponding place_part: " << place_part << endl);
                    
                    place_part_name = std::string(place_part.name);
                    part_in_inventory = binInventory.find_part(current_inventory, place_part_name, pick_part, partnum_in_inventory);
                    if (!part_in_inventory) {
                        ROS_WARN("this part not found in inventory; marking it unavailable to fill");
                        parts_checklist[desired_part_id] = true;
                    }
                    ROS_INFO_STREAM("chosen pick_part: " << endl << pick_part << endl);

    
                    //plan ahead--is both acquisition  and  drop-off  feasible?
                    go_on = robotBehaviorInterface.evaluate_key_pick_and_place_poses(pick_part,place_part);
                    if (!go_on) {
                        ROS_WARN("could not compute key pickup and place poses for this part source and destination");
                    }    
                    
                    if (go_on) { //so far, so good: try acquiring part and moving to viewing pose:
                        ROS_INFO_STREAM("attempting to acquire and place part"); 
                        go_on = shipmentFiller.get_part_and_prepare_place_in_box(current_inventory, place_part); 
                    }
                    
                    if (go_on) {
                        ros::Duration(1.0).sleep(); //wait for settling before observing pose
                        observed_part=place_part; //fills in part name;the rest is wrong
                            //see if part is viewable:
                        
                        if(gripperInterface.isGripperAttached()) {
                          //observed_part is a reference var--will get repopulated
                          //use grasp pose to recompute key dropoff poses, if possible
                            bool did_observe_grasped_part=false;
                            
                            
                            did_observe_grasped_part = boxInspector.get_grasped_part_pose_wrt_world(observed_part); //updates observed_part
                            if (did_observe_grasped_part) {
                                ROS_INFO_STREAM("observed grasped part: " << endl << observed_part << endl);
                                ROS_WARN("calling re_evaluate_approach_and_place_poses");
                                if (!robotBehaviorInterface.re_evaluate_approach_and_place_poses(observed_part, place_part)) {
                                    //go back to original key poses:
                                    robotBehaviorInterface.evaluate_key_pick_and_place_poses(pick_part,place_part);
                                    ROS_WARN("re-evaluation of dropoff poses was not successful;  using open-loop plan");
                                }         
                            }
                            else { 
                                ROS_WARN("could not observe grasped part; using open-loop dropoff plan");
                                //retain original  key poses
                            }
                        }

                        else {
                            ROS_WARN("dropped part");
                        }
                        if (!go_on) {
                            //can't reach corrected poses.  May as well just drop or discard the part
                            ROS_WARN("something failed; discarding grasped part");
                            robotBehaviorInterface.discard_grasped_part(place_part);
                            //problem_model_indices.push_back(i_model); //add to list of problem parts
                            go_on = false; //redundant
                        }

                        if (go_on) {
                            ROS_INFO("using best estimate approach and place poses to place part in box, no release");
                            go_on = robotBehaviorInterface.place_part_in_box_from_approach_no_release(place_part);
                            //remove this part from missing-parts vector manually--in case operating blind
                        }

                        //check if this part passes quality inspection:
                        if (shipmentFiller.get_bad_part_Q1(bad_part)) {
                            ROS_WARN("this part is bad; discard it");
                            //do discard here
                            robotBehaviorInterface.discard_grasped_part(bad_part);
                        } else {
                            //release part and retract arm
                            ROS_INFO("quality sensor did not declare part as bad; leave it");
                            robotBehaviorInterface.release_and_retract();
                            
                            missing_models_wrt_world.erase(missing_models_wrt_world.begin()+missing_part_index);
                            satisfied_models_wrt_world.push_back(current_model);
                            part_indices_precisely_placed.push_back(desired_part_id); //Not sure about the index. Doing this to prevent segfaults. 
                            parts_checklist[desired_part_id] = true;
                            //assuming that placement is accurate, removed current model from missing models vector and placed it in satisfied models. Also marked it as 'tried' in the parts checklist 
                            //ROS_DEBUG("Deleted missing model cuz no box inspector update");
                        }
                    }
                }
            }//concludes USE_CURRENT_BOX case for one pick/place operation at Q1 station
            else {
                ROS_WARN("ERROR--unexpected return code from optimizer");
            }
        }


        //here is the case for working on shipment at Q2
        if (have_active_shipment_Q2) { //needs lots of work
            ROS_INFO("working on order at Q2");
            //wait for box to arrive at Q2 
            ROS_INFO("checking if box is in position");
            while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_ESTIMATED_AT_Q2 &&
                    (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SEEN_AT_Q2)) {
                ros::spinOnce();
                if (!conveyorInterface.drone_depot_sees_box()) waiting_on_drone=false; //sprinkle these throughout                
                ros::Duration(0.1).sleep();
                nprint++;     
                if (nprint % 10 == 0) {
                    ROS_INFO("waiting for conveyor to advance a box to Q2...");
                }
            }
            ROS_WARN("BOX IS AT Q2; SKIPPING Q2 INSPECTION AND OPERATIONS");
            //do a box inspection at Q2:
            //also do a quality inspection at Q2;
            //for a bad part, put this in the "orphaned" list (and remove from any other list)
            //then ask for optimized shipment update;
            //  expect cases: int16 USE_CURRENT_BOX, PRIORITY_SHIP_THIS, PRIORITY_LOAD_NEXT, or SHIP_NO_LABEL
            optimizer_msg.request.giving_up = optimizer_func::optimizer_msgsRequest::GIVING_UP;
            optimizer_msg.request.inspection_site = optimizer_func::optimizer_msgsRequest::Q2_STATION;
            ROS_INFO("sending request to optimizer from Q2");
            optimizer_client.call(optimizer_msg);
            //Q2_shipment_name = std::string(shipment.shipment_type);        
            //TEMP DEBUG: treat this as "PRIORITY_LOAD_NEXT"

            if (optimizer_msg.response.decision == optimizer_func::optimizer_msgsResponse::PRIORITY_LOAD_NEXT) {
                ROS_INFO("optimizer returned PRIORTY_LOAD_NEXT; advancing next box to Q1");
                //prep for focus back to Q1:
                conveyorInterface.move_new_box_to_Q1();
                have_active_shipment_Q2 = false;
                Q1_shipment_name = no_shipment_name;
                prior_Q1_shipment_name = Q1_shipment_name;
                have_active_shipment_Q1 = false;
                drone_shipment_name = Q2_shipment_name; //this is the label to use for drone shipping
                drone_shipment_queue.push(Q2_shipment);  //remember this shipment
            } else if (optimizer_msg.response.decision == optimizer_func::optimizer_msgsResponse::USE_CURRENT_BOX) {
                ROS_WARN("commanded to use current box...UNEXPECTED; THIS  CODE NOT READY");
                //xxx FIX THIS: replace with q2 inspection handling only, else ship as-is, depending on optimizer response
                conveyorInterface.move_new_box_to_Q1();
                have_active_shipment_Q2 = false;
                Q1_shipment_name = no_shipment_name;
                prior_Q1_shipment_name = Q1_shipment_name;
                have_active_shipment_Q1 = false;
                drone_shipment_name = Q2_shipment_name; //this is the label to use for drone shipping

            }

        }

        //CHECK ON DEPOT STATUS: ship a box, if present;
        ros::spinOnce(); //need to update callbacks to get depot status
        if(conveyorInterface.drone_depot_sees_box()) {
            ROS_WARN("true conveyorInterface.drone_depot_sees_box()");
            if (waiting_on_drone) ROS_WARN("and waiting on drone");
            else ROS_WARN("need to call drone");
        }
        if (conveyorInterface.drone_depot_sees_box()&& (!waiting_on_drone)) {
            ROS_INFO("shipment seen at drone depot--making service call to  drone");
            drone_shipment=drone_shipment_queue.front();
            drone_shipment_queue.pop();
            shipmentFiller.set_drone_shipment_name(drone_shipment);
            reported_shipment_to_drone = false;
            while (!reported_shipment_to_drone) { //be persistent!
               reported_shipment_to_drone = shipmentFiller.report_shipment_to_drone();
               ros::spinOnce();
               ros::Duration(0.1).sleep();
            }
            waiting_on_drone=true;
        }
        //else ROS_INFO("conveyorInterface does not  report seeing a box at depot");

        if (waiting_on_drone) ROS_WARN("drone has  not yet picked up shipment");
        ros::spinOnce();        
        if (!conveyorInterface.drone_depot_sees_box()) waiting_on_drone=false;


    }
    return 0; //while ros::OK()
}

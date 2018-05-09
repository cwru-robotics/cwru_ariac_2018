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
#include <kuka_move_as/RobotBehaviorInterface.h>
#include<bin_inventory/bin_inventory.h>
#include<box_inspector/box_inspector.h>
#include<conveyor_as/ConveyorInterface.h>
#include<optimizer_func/optimizer_msgs.h>

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

int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "shipment_filler"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    //here are a bunch of useful objects.  Most of these are also instantiated  "ShipmentFiller"
    //still deciding what and how much to expose at this top level
    int ans;
    inventory_msgs::Part bad_part;
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
    optimizer_func::optimizer_msgs optimizer_msg;

    ros::ServiceClient optimizer_client = nh.serviceClient<optimizer_func::optimizer_msgs>("optimizer");
    while (!optimizer_client.exists()) {
        ROS_INFO("waiting for optimization service...");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to optimization service");
    //optimizer_msg.request.
    /*
     bool BoxInspector::update_inspection(vector<osrf_gear::Model> desired_models_wrt_world,
        vector<osrf_gear::Model> &satisfied_models_wrt_world,
        vector<osrf_gear::Model> &misplaced_models_actual_coords_wrt_world,
        vector<osrf_gear::Model> &misplaced_models_desired_coords_wrt_world,
        vector<osrf_gear::Model> &missing_models_wrt_world,
        vector<osrf_gear::Model> &orphan_models_wrt_world)
     */
    osrf_gear::Shipment shipment_loaded, shipment_orphaned, shipment_missing, shipment_reposition, empty_shipment;

    inventory_msgs::Part pick_part, place_part, observed_part, remove_part;
    inventory_msgs::Inventory current_inventory;
    osrf_gear::Shipment shipment, Q1_shipment, Q2_shipment; //keep track of shipments at Q1 and and Q2
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
    //vector<int> problem_model_acquisition_indices;
    //vector<int> problem_model_reposition_indices;  
    vector<int> problem_model_indices;

    //inventory_msgs::Part current_part;
    int bin_num, partnum;
    geometry_msgs::PoseStamped part_pose;
    int a; //remove after debyg
    bool got_shipment = false;
    bool successfully_filled_order = false;
    bool init_pack_shipment = false;
    bool replaced_faulty_parts = false;
    bool adjusted_part_locations = false;
    //bool corrected_dropped_part=false;
    bool reported_shipment_to_drone = false;
    bool advanced_shipment_on_conveyor = false;
    bool go_on = true;

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
    shipmentFiller.modelvec_to_shipment(current_shipment_name, satisfied_models_wrt_world, boxInspector.NOM_BOX1_POSE_WRT_WORLD, shipment_loaded);
    shipmentFiller.modelvec_to_shipment(current_shipment_name, orphan_models_wrt_world, boxInspector.NOM_BOX1_POSE_WRT_WORLD, shipment_orphaned);
    shipmentFiller.modelvec_to_shipment(current_shipment_name, missing_models_wrt_world, boxInspector.NOM_BOX1_POSE_WRT_WORLD, shipment_missing);
    shipmentFiller.modelvec_to_shipment(current_shipment_name, misplaced_models_actual_coords_wrt_world, boxInspector.NOM_BOX1_POSE_WRT_WORLD, shipment_reposition);
    optimizer_msg.request.loaded = shipment_loaded;
    optimizer_msg.request.orphaned = shipment_missing;
    optimizer_msg.request.reposition = shipment_reposition;
    optimizer_msg.request.giving_up = optimizer_func::optimizer_msgsRequest::NOT_GIVING_UP;

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
    while (Q1_shipment_name == no_shipment_name) {
        ROS_INFO("waiting for first order...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        optimizer_client.call(optimizer_msg);
        shipment = optimizer_msg.response.shipment;
        string rtn_name(shipment.shipment_type);
        Q1_shipment_name = rtn_name;
        ROS_INFO_STREAM("optimizer returned shipment name: " << Q1_shipment_name << endl);
    }
    ROS_INFO("Got first shipment: ");
    Q1_shipment = shipment; //make a copy of this shipment here, noting it is associated with station Q1
    ROS_INFO_STREAM("initial shipment: " << endl << Q1_shipment << endl);

    ROS_INFO("computing estimated destination poses based on expected box pose");
    boxInspector.compute_shipment_poses_wrt_world(Q1_shipment, boxInspector.NOM_BOX1_POSE_WRT_WORLD, desired_models_wrt_world);
    successfully_filled_order = false;
    bool have_active_shipment_Q1 = true;

    //acquire first part while waiting on box; move this part to camera inspection above box
    int num_models = desired_models_wrt_world.size();
    int i_model = 0;
    current_model = desired_models_wrt_world[i_model]; //try to get the first model of this shipment ready
    //build "part" description for destination
    shipmentFiller.model_to_part(current_model, place_part, inventory_msgs::Part::QUALITY_SENSOR_1);
    std::string part_name(place_part.name);
    ROS_INFO_STREAM("attempting to acquire and place part: " << place_part << endl);

    //try (up to three times) to get this part and show it to box1 camera; if NG, try next part
    while ((i_model < num_models)&&!shipmentFiller.get_part_and_prepare_place_in_box(current_inventory, place_part)) {
        ROS_WARN("giving up on model index %d; trying next model", i_model); //very unlikely to happen
        problem_model_indices.push_back(i_model);
        i_model++;
        if (i_model < num_models) {
            current_model = desired_models_wrt_world[i_model]; //try to get the first model of this shipment ready
            //build "part" description for destination
            shipmentFiller.model_to_part(current_model, place_part, inventory_msgs::Part::QUALITY_SENSOR_1);
        }
    }
    if (i_model < num_models) {
        ROS_INFO("grasped part index %d and holding under box1 cam", i_model);
    } else {
        ROS_WARN("something is horribly wrong; could not grasp acquire ANY parts  from this order!");
        Q1_shipment = empty_shipment;
    }

    //once here, part is presumably grasped and under box1 cam; I will  not check if initial grasp attempts were all unsuccessful

    //have to wait for box to show up at Q1:
    int nprint = 0;
    while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_ESTIMATED_AT_Q1 &&
            (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SEEN_AT_Q1)) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        nprint++;
        if (nprint % 10 == 0) {
            ROS_INFO("waiting for conveyor to advance a box to Q2...");
            ROS_INFO("waiting for conveyor to advance a box to Q1...");
        }
    }

    //recompute key poses for placement
    //place part, release and retract
    //do box inspection in prep for next call to shipment optimizer    
    //get grasp transform from camera
    boxInspector.get_grasped_part_pose_wrt_world(observed_part);
    ROS_WARN("conveyor halted; get pose of grasped  part");
    ROS_INFO_STREAM("observed grasped part: " << endl << observed_part << endl);

    //update box pose,  if possible              
    if (boxInspector.get_box_pose_wrt_world(box_pose_wrt_world)) {
        ROS_INFO_STREAM("box seen at: " << box_pose_wrt_world << endl);
        //update desired part poses w/rt world:
        boxInspector.compute_shipment_poses_wrt_world(shipment, box_pose_wrt_world, desired_models_wrt_world);
    }

    //update destination for currently held part:
    current_model = desired_models_wrt_world[i_model]; //try to get the first model of this shipment ready
    ROS_INFO_STREAM("current model w/ placement coords w/rt world: " << endl << current_model << endl);
    //build "part" description for destination
    shipmentFiller.model_to_part(current_model, place_part, inventory_msgs::Part::QUALITY_SENSOR_1);
    ROS_INFO_STREAM("re-expressed as a Part object: " << endl << place_part << endl);
    ROS_WARN("calling re_evaluate_approach_and_place_poses");
    go_on = robotBehaviorInterface.re_evaluate_approach_and_place_poses(observed_part, place_part);

    if (!go_on) {
        //can't reach corrected poses.  May as well just drop or discard the part
        ROS_WARN("something failed; discarding grasped part");
        robotBehaviorInterface.discard_grasped_part(place_part);
        problem_model_indices.push_back(i_model); //add to list of problem parts
        go_on = false; //redundant
    }

    if (go_on) {
        ROS_INFO("using newly adjusted approach and place poses to place part in box, no release");
        go_on = robotBehaviorInterface.place_part_in_box_from_approach_no_release(place_part);
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
    }


    //END OF LONG PREAMBLE

    while (ros::ok()) {
        //Q1 inspection:
        //if (Q1_shipment_name!=no_shipment_name) 
        if (have_active_shipment_Q1) { //major block: have active shipment at Q1; work on it!
            //do a box inspection to see how we are  doing:
            if (boxInspector.get_box_pose_wrt_world(box_pose_wrt_world)) {
                //update desired part poses w/rt world:
                boxInspector.compute_shipment_poses_wrt_world(shipment, box_pose_wrt_world, desired_models_wrt_world);
            }

        }
        //inspect the box:
        boxInspector.update_inspection(desired_models_wrt_world, satisfied_models_wrt_world,
                misplaced_models_actual_coords_wrt_world, misplaced_models_desired_coords_wrt_world,
                missing_models_wrt_world, orphan_models_wrt_world);

        //xxx FINISH ME: do a Q1 camera check; a bad part should be classified as an orphaned  part

        //populate a service message for optimizer:    
        shipmentFiller.modelvec_to_shipment(current_shipment_name, satisfied_models_wrt_world, boxInspector.NOM_BOX1_POSE_WRT_WORLD, shipment_loaded);
        shipmentFiller.modelvec_to_shipment(current_shipment_name, orphan_models_wrt_world, boxInspector.NOM_BOX1_POSE_WRT_WORLD, shipment_orphaned);
        shipmentFiller.modelvec_to_shipment(current_shipment_name, missing_models_wrt_world, boxInspector.NOM_BOX1_POSE_WRT_WORLD, shipment_missing);
        shipmentFiller.modelvec_to_shipment(current_shipment_name, misplaced_models_actual_coords_wrt_world, boxInspector.NOM_BOX1_POSE_WRT_WORLD, shipment_reposition);
        optimizer_msg.request.loaded = shipment_loaded;
        optimizer_msg.request.orphaned = shipment_missing;
        optimizer_msg.request.reposition = shipment_reposition;
        optimizer_msg.request.giving_up = optimizer_func::optimizer_msgsRequest::NOT_GIVING_UP;
        //send a service request:
        optimizer_client.call(optimizer_msg);
        //check if current box should continue to be used:
        if (optimizer_msg.response.decision == optimizer_func::optimizer_msgsResponse::USE_CURRENT_BOX) {
            ROS_INFO("optimizer says to use this box");
            //do actions in this order: 1) unload orphaned parts; 2) adjust imprecisely placed parts; 3) load new parts
            //note: advised shipment might be NEW:
            Q1_shipment = optimizer_msg.response.shipment;
            //update inspection:
            boxInspector.update_inspection(desired_models_wrt_world, satisfied_models_wrt_world,
                    misplaced_models_actual_coords_wrt_world, misplaced_models_desired_coords_wrt_world,
                    missing_models_wrt_world, orphan_models_wrt_world);

            //highest priority: remove an orphaned part
            if (orphan_models_wrt_world.size() > 0) {
                //remove orphaned parts:  FIX ME! watch out for infinite loop
                current_model = orphan_models_wrt_world[0];
                shipmentFiller.model_to_part(current_model, remove_part, inventory_msgs::Part::QUALITY_SENSOR_1);
                robotBehaviorInterface.pick_part_from_box(remove_part);
                robotBehaviorInterface.discard_grasped_part(remove_part);
            }
                //second priority: adjust a part position
                /*
                else if (misplaced_models_actual_coords_wrt_world.size()>0) {
                    ROS_WARN("should do part relocation here...");
                }      */

                //third priority: get a new part and try to place in box
            else if (missing_models_wrt_world.size() > 0) { //get a new part and place it in box
                //do the sequence to pick and place:
                current_model = missing_models_wrt_world[0];
                shipmentFiller.model_to_part(current_model, place_part, inventory_msgs::Part::QUALITY_SENSOR_1);
                go_on = shipmentFiller.get_part_and_prepare_place_in_box(current_inventory, place_part);
                if (go_on) {
                    boxInspector.get_grasped_part_pose_wrt_world(observed_part);
                    ROS_INFO_STREAM("observed grasped part: " << endl << observed_part << endl);
                    go_on = robotBehaviorInterface.re_evaluate_approach_and_place_poses(observed_part, place_part);
                    if (!go_on) {
                        //can't reach corrected poses.  May as well just drop or discard the part
                        ROS_WARN("something failed; discarding grasped part");
                        robotBehaviorInterface.discard_grasped_part(place_part);
                        //problem_model_indices.push_back(i_model); //add to list of problem parts
                        go_on = false; //redundant
                    }

                    if (go_on) {
                        ROS_INFO("using newly adjusted approach and place poses to place part in box, no release");
                        go_on = robotBehaviorInterface.place_part_in_box_from_approach_no_release(place_part);
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
                    }
                }
            }
        }//concludes USE_CURRENT_BOX case for one pick/place operation

        else if (optimizer_msg.response.decision == optimizer_func::optimizer_msgsResponse::ADVANCE_THIS_BOX_TO_Q2) {
            Q2_shipment = Q1_shipment; //same shipment specifications,  but now at Q2
            Q1_shipment = empty_shipment;
            have_active_shipment_Q1 = false;

            conveyorInterface.move_box_Q1_to_Q2();
            //wait for box to arrive:
            nprint = 0;
            while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_ESTIMATED_AT_Q2 &&
                    (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SEEN_AT_Q2)) {
                ros::spinOnce();
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

            //TEMP DEBUG: treat this as "PRIORITY_LOAD_NEXT"
            ROS_INFO("moving a new box to Q1");
            conveyorInterface.move_new_box_to_Q1();
        }

        /*
                advanced_shipment_on_conveyor =
                        shipmentFiller.advance_shipment_on_conveyor(DRONE_DOCK_LOCATION_CODE);
                //send notice to drone:

                reported_shipment_to_drone = shipmentFiller.report_shipment_to_drone();
                //send robot to waiting pose; update inventory
                if (binInventory.update()) {
                    binInventory.get_inventory(current_inventory);
                }

         */
        ROS_WARN("stopping after single shipment...FIX ME!");
        return 0;

    } //while ros::OK()
}

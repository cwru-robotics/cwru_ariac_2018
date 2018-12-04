//unload_box.cpp:
// moves a box under camera, then removes sensed parts from box

//use a RobotBehaviorInterface object to communicate with the robot behavior action server
#include <robot_behavior_interface/RobotBehaviorInterface.h>

//we define a message type for a "part" that includes name, pose and associated location code (e.g. bin # or box location)
#include<inventory_msgs/Part.h>

//a "box inspector" object can compare a packing list to a logical camera image to see how we are doing
#include<box_inspector/box_inspector.h>

//conveyor interface communicates with the conveyor action server
#include<conveyor_as/ConveyorInterface.h>


const double COMPETITION_TIMEOUT=500.0; // need to  know what this is for the finals;
// want to ship out partial credit before time runs out!

osrf_gear::Order g_order;
bool g_got_order = false;

void model_to_part(osrf_gear::Model model, inventory_msgs::Part &part, unsigned short int location) {
    part.name = model.type;
    part.pose.pose = model.pose;
    part.location = location; //by default
}

// Listening for the Orders from ARIAC
void orderCallback(const osrf_gear::Order::ConstPtr& msg) {

  g_order = *msg;
  g_got_order = true;
  ROS_INFO("Received order %s with %i shipment%s", msg->order_id.c_str(), (int) msg->shipments.size(), msg->shipments.size() == 1 ? "": "s");
  ROS_INFO_STREAM(g_order);
}

int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "box_unloader"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    int ans;
    
 
    ROS_INFO("instantiating a RobotBehaviorInterface");
    RobotBehaviorInterface robotBehaviorInterface(&nh); //shipmentFiller owns one as well

    ROS_INFO("instantiating a ConveyorInterface");
    ConveyorInterface conveyorInterface(&nh);
   
    ROS_INFO("instantiating a BoxInspector");
    BoxInspector boxInspector(&nh);

    //instantiate an object of appropriate data type for our move-part commands
    inventory_msgs::Part current_part,desired_part;

    geometry_msgs::PoseStamped box_pose_wrt_world;  //camera sees box, coordinates are converted to world coords
    
    bool status;    
    int nparts;
    
    ros::Subscriber sub = nh.subscribe("ariac/orders", 5, orderCallback);
    ROS_INFO("waiting for order...");
    while (!g_got_order) {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        if  (!g_got_order) ROS_INFO("waiting");
        
    }

    //for box inspector, need to define multiple vectors for args, 
    //box inspector will identify parts and convert their coords to world frame
    //in the present example, desired_models_wrt_world is left empty, so ALL observed parts will be considered "orphaned"
        vector<osrf_gear::Model> desired_models_wrt_world;
        vector<osrf_gear::Model> satisfied_models_wrt_world;
        vector<osrf_gear::Model> misplaced_models_actual_coords_wrt_world;
        vector<osrf_gear::Model> misplaced_models_desired_coords_wrt_world;
        vector<osrf_gear::Model> missing_models_wrt_world;
        vector<osrf_gear::Model> orphan_models_wrt_world;
        vector<int> part_indices_missing;
        vector<int> part_indices_misplaced;
        vector<int> part_indices_precisely_placed;

    
    //use conveyor action  server for multi-tasking
    ROS_INFO("getting a box into position: ");
    int nprint = 0;
    conveyorInterface.move_new_box_to_Q1();  //member function of conveyor interface to move a box to inspection station 1
    while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SEEN_AT_Q1) {
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
    }
    else {
        ROS_WARN("no box seen.  something is wrong! I quit!!");
        exit(1);
    }
    
    // if survive to here, then box is at Q1 inspection station; 
    
    //compute desired  part poses w/rt world, given box location:
    //void BoxInspector::compute_shipment_poses_wrt_world(osrf_gear::Shipment shipment_wrt_box,
    //    geometry_msgs::PoseStamped box_pose_wrt_world,
    //    vector<osrf_gear::Model> &desired_models_wrt_world)
    boxInspector.compute_shipment_poses_wrt_world(g_order.shipments[0],box_pose_wrt_world,desired_models_wrt_world);
     
    //inspect the box and classify all observed parts
    boxInspector.update_inspection(desired_models_wrt_world,
        satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
        orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
        part_indices_precisely_placed);
    ROS_INFO("orphaned parts in box: ");
    nparts = orphan_models_wrt_world.size();
    ROS_INFO("num parts seen in box = %d",nparts);
    for (int i=0;i<nparts;i++) {
       ROS_INFO_STREAM("orphaned  parts: "<<orphan_models_wrt_world[i]<<endl);
    }
    
    

    if (boxInspector.get_bad_part_Q1(current_part)) {
        ROS_INFO("found bad part: ");
        ROS_INFO_STREAM(current_part<<endl);
        
        cout<<"enter 1 to attempt to remove bad part: "; //poor-man's breakpoint
        cin>>ans;        
        status = robotBehaviorInterface.pick_part_from_box(current_part);
        //and discard it:
        status = robotBehaviorInterface.discard_grasped_part(current_part);


        
        //XXX YOU should attempt to remove the defective part here...
        // see example, line 139, robotBehaviorInterface.pick_part_from_box();

    //use the robot action server to acquire and dispose of the specified part in the box:
    }    

    //after removing the bad part, re-inspect the box:
    boxInspector.update_inspection(desired_models_wrt_world,
        satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
        orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
        part_indices_precisely_placed);
    ROS_INFO("orphaned parts in box: ");
    nparts = orphan_models_wrt_world.size();
    ROS_INFO("num parts seen in box = %d",nparts);
    for (int i=0;i<nparts;i++) {
       ROS_INFO_STREAM("orphaned  parts: "<<orphan_models_wrt_world[i]<<endl);
    }


    // move robot to grasp and discard each part

    //start w/ first part

    //box inspector sees "model", defined in osrf_gear; convert this to our datatype "Part"
    //void model_to_part(osrf_gear::Model model, inventory_msgs::Part &part, unsigned short int location) 
    //SHOULD do this for ALL orphaned parts
    
    
    nparts = orphan_models_wrt_world.size();
    while (nparts> 0) {
       model_to_part(orphan_models_wrt_world[0], current_part, inventory_msgs::Part::QUALITY_SENSOR_1);

       //use the robot action server to grasp part in the box:
       status = robotBehaviorInterface.pick_part_from_box(current_part);
        //and discard it:
        status = robotBehaviorInterface.discard_grasped_part(current_part);    
        
       boxInspector.update_inspection(desired_models_wrt_world,
        satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
        orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
        part_indices_precisely_placed);
       //this loop focuses on orphans, which includes bad parts
        nparts = orphan_models_wrt_world.size();

    }

    //SHOULD REPEAT FOR ALL THE PARTS IN THE BOX
    //ALSO, WATCH OUT FOR NO PARTS IN THE BOX--ABOVE WILL CRASH
    ROS_INFO("done removing orphans; attempt part relocations, as necessary");
        cout<<"enter 1 to re-inspect: "; //poor-man's breakpoint
        cin>>ans;  
       boxInspector.update_inspection(desired_models_wrt_world,
        satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
        orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
        part_indices_precisely_placed);
        nparts = orphan_models_wrt_world.size();        
         
    int nparts_misplaced =   misplaced_models_actual_coords_wrt_world.size();
    ROS_INFO("found %d misplaced parts",nparts_misplaced);
    while (nparts_misplaced>0) {
        model_to_part(misplaced_models_actual_coords_wrt_world[0], current_part, inventory_msgs::Part::QUALITY_SENSOR_1);
        //adjust_part_location_no_release(Part sourcePart, Part destinationPart, double timeout = MAX_ACTION_SERVER_WAIT_TIME);
        int index_des_part = part_indices_misplaced[0];
        model_to_part(desired_models_wrt_world[index_des_part], desired_part, inventory_msgs::Part::QUALITY_SENSOR_1);
        ROS_INFO("move part from: ");
        ROS_INFO_STREAM(current_part);
        ROS_INFO("move part to: ");
        ROS_INFO_STREAM(desired_part);
        cout<<"enter  1 to reposition part"<<endl;
        cin>>ans;
       //use the robot action server to grasp part in the box:
        status = robotBehaviorInterface.pick_part_from_box(current_part); 
        
        //following fnc works ONLY if part is already grasped:
        status = robotBehaviorInterface.adjust_part_location_no_release(current_part,desired_part);
        status = robotBehaviorInterface.release_and_retract();
        
       boxInspector.update_inspection(desired_models_wrt_world,
        satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
        orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
        part_indices_precisely_placed);
        nparts_misplaced = misplaced_models_actual_coords_wrt_world.size();
       
    }

    
    ROS_INFO("all done; goodbye, world");
            return 0;
    //here's an oddity: this node runs to completion.  But sometimes, Linux complains bitterly about
    // *** Error in `/home/wyatt/ros_ws/devel/lib/shipment_filler/unload_box': corrupted size vs. prev_size: 0x000000000227c7c0 ***
    // don't know why.  But does not seem to matter.  If anyone figures this  out, please let me know.
}

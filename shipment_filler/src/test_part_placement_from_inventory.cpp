//test fnc: discard ALL parts in bin inventory!!

//#include<order_manager/order_manager.h>
//#include<shipment_filler/ShipmentFiller.h>

#include <robot_behavior_interface/RobotBehaviorInterface.h>
#include<bin_inventory/bin_inventory.h>
#include<box_inspector/box_inspector.h>

int ans;
void part_to_model(inventory_msgs::Part part, osrf_gear::Model &model) {
	model.type=part.name;
	model.pose=part.pose.pose;

}

void set_part_vals(Part pick_part,Part &place_part) {
    geometry_msgs::PoseStamped place_pose;
    //a hard-coded drop-off location: approx centered in box1
    //Q1 box pose: ~ centered--> x,y,z = 0.612, 0.612, 0.588
    place_pose.pose.position.x = 0.57; //these values are centered under box cam1
    place_pose.pose.position.y = 0.61; //+= 0.01; //= 3.13; //0; //3.300;
    place_pose.pose.position.z = 0.59; //0; //0.750;    
    place_pose.pose.orientation.x=0;
    place_pose.pose.orientation.y=0;
    place_pose.pose.orientation.z= 0;
    place_pose.pose.orientation.w=1;    

    place_part = pick_part;  //copy over data, replace what needs to  be  replaced
    place_pose.header.frame_id="world"; //should already be this    
    place_part.pose = place_pose;
    place_part.location= inventory_msgs::Part::QUALITY_SENSOR_1; 
}



int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "place_from_inventory_tester"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    //ROS_INFO("main: instantiating an object of type OrderManager");
    //OrderManager orderManager(&nh);

    RobotBehaviorInterface robotBehaviorInterface(&nh);
    
    inventory_msgs::Part pick_part,place_part,observed_part;
    inventory_msgs::Inventory inventory_msg;

    BoxInspector boxInspector(&nh);

    ROS_INFO("main: instantiating an object of type BinInventory");
    BinInventory binInventory(&nh);  
    binInventory.update();

    //void BinInventory::get_inventory(inventory_msgs::Inventory &inventory_msg) {
    binInventory.get_inventory(inventory_msg);
    binInventory.print_inventory_msg();
    osrf_gear::Model model,misplaced_model_actual_coords,misplaced_model_desired_coords;
    vector<osrf_gear::Model> desired_models_wrt_world;
    desired_models_wrt_world.clear();
    /*     part.location = bin_num;
     part.pose = part_pose;
     part.name = part_name.c_str();*/
    //could start from 2 to skip the gear parts
    bool go_on = true;
    for (int i_part_type=1;i_part_type<=NUM_PART_TYPES;i_part_type++) {
        std::string part_name = part_id_to_name_mappings[i_part_type];
        ROS_INFO_STREAM("attempting to remove parts  of  type "<<part_name<<endl);
        int nparts = binInventory.num_parts(i_part_type);
        ROS_INFO_STREAM("found "<<nparts<<" of these"<<endl);
        for (int ipart=0;ipart<nparts;ipart++) {
            go_on = true;
            ROS_INFO("removing item %d of part_id %d",ipart, i_part_type);
              unsigned short int bin_num =   inventory_msg.inventory[i_part_type].bins[ipart];
              geometry_msgs::PoseStamped part_pose = inventory_msg.inventory[i_part_type].part_stamped_poses[ipart];
              ROS_INFO_STREAM("found part in bin "<<bin_num<<" at pose "<<part_pose<<endl);
              pick_part.location = bin_num;
              pick_part.pose = part_pose;
              pick_part.name = part_name.c_str();
              ROS_INFO_STREAM("pick_part: "<<pick_part<<endl);
              
              //ROS_INFO_STREAM("attempting pick command for part "<<pick_part<<endl);
                //pick_part.location= inventory_msgs::Part::BIN2;
              ROS_INFO("attempting pick...");
               if(!robotBehaviorInterface.pick_part_from_bin(pick_part)) {
                  ROS_INFO("pick failed");
                  //gripperInterface_.release();     
                  go_on=false;
               }               
              if (go_on) {
                ROS_INFO("attempting to place part: ");
                set_part_vals(pick_part,place_part);
                ROS_INFO_STREAM("part to be placed: "<<place_part<<endl);
              //place_part_in_box_no_release(Part part,double timeout)
              //bool RobotBehaviorInterface::move_part_to_approach_pose(inventory_msgs::Part part,double timeout) {
                if(!robotBehaviorInterface.move_part_to_approach_pose(place_part)) {
                  ROS_INFO("placement failed");
                  //gripperInterface_.release();     
                  go_on=false;
               }   
              }
              if (go_on) {
                  ROS_INFO("getting observed pose of part to compute grasp transform");
                  ROS_INFO("pause first, for settling time");
                  ros::Duration(2.0).sleep();
                boxInspector.get_observed_part_pose(place_part, observed_part);
                //cout<<"enter 1: ";
                //cin>>ans;
                //when above fnc call is working, call the following
                ROS_WARN("should now call re_evaluate_approach_and_place_poses...or put in shipmentFiller fnc");               
                go_on = robotBehaviorInterface.re_evaluate_approach_and_place_poses(observed_part,place_part);              
              }
              if (go_on) {
                if(!robotBehaviorInterface.place_part_in_box_from_approach_no_release(place_part)) {
                  ROS_INFO("placement failed");
                  //gripperInterface_.release();     
                  go_on=false;
               }         
              }
              ROS_INFO("desired part x,y,z = %f, %f, %f",place_part.pose.pose.position.x,place_part.pose.pose.position.y,place_part.pose.pose.position.z );

              cout<<"enter  1: ";
              cin>>ans;
              if (go_on) {
                 robotBehaviorInterface.discard_grasped_part(place_part);
              }
              
              //dummy test: just claim part is observed to be in exact location
              // replace this with actual observation from camera
              /*
                if(!robotBehaviorInterface.adjust_part_location_no_release(place_part,place_part)) {
                  ROS_INFO("placement failed");
                  //gripperInterface_.release();     
                  break;
               }    
              */
              
              //DEMO OF "RELEASE" FNC:  need to remove this, or parts will pile up!!
              //command to release a part--waits for max of 5 seconds for confirming release, else returns "false"
              /*
              if(!robotBehaviorInterface.release(5)) {
                  ROS_WARN("something is wrong!  unsuccessful releasing part after 5 seconds of trying");
              } 
              /**/
              
    /*
    ROS_INFO("double check starts");          

    //AUGMENT HERE!!!
    part_to_model(place_part,model); 
    //only for debug//
    //Ignore below, better implementation in shipment filler
    //bool pre_dropoff_check = boxInspector.pre_dropoff_check(desired_models_wrt_world,misplaced_model_desired_coords, misplaced_model_actual_coords);
    //while(!pre_dropoff_check) {
    	inventory_msgs::Part part_wrong_pose, part_desired_pose;
    	part_wrong_pose=place_part;
    	part_desired_pose=place_part;
    	part_wrong_pose.pose.pose=misplaced_model_actual_coords.pose;
    	part_desired_pose.pose.pose=misplaced_model_desired_coords.pose;
    	//perturbation function with part_wrong_pose and part_desired_pose as args
      	ROS_INFO_STREAM("part at wrong pose: " <<part_wrong_pose);
      	ROS_INFO_STREAM("part at correct pose: "<<part_desired_pose);
    	//pre_dropoff_check=boxInspector.pre_dropoff_check(desired_models_wrt_world,misplaced_model_desired_coords, misplaced_model_actual_coords);
   

    // check box1 camera for part pose; convert to world coords;
    // perform adjustment, as necessary, before placing part
    // confirm part is within tolerance of goal
    // re-grasp and correct,  as necessary              
              
              ROS_INFO("discarding part");
               if(!robotBehaviorInterface.discard_grasped_part(place_part)) {
                  ROS_INFO("discard failed");
                  //gripperInterface_.release();     
                  break;
               }  */                    
        } 
    }

}

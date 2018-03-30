//main pgm that illustrates use of box-inspector library
#include<box_inspector/box_inspector.h>

osrf_gear::Shipment g_shipment_wrt_box;
bool g_order_received = false;

void order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
    if (!g_order_received) {
       ROS_INFO_STREAM("Received order:\n" << *order_msg);
       osrf_gear::Order order;
       order = *order_msg;
       g_shipment_wrt_box = order.shipments[0]; //arbitrarily, choose the first  shipment of received order
       
       g_order_received = true;
    }
}



int main(int argc, char ** argv) {
  ros::init(argc, argv, "box_inspector");
  ros::NodeHandle nh;
  //populate the following based on shipment info:
  vector<osrf_gear::Model> desired_models_wrt_world;

  //vectors to hold the inspection  results:
   vector<osrf_gear::Model> satisfied_models_wrt_world;
   vector<osrf_gear::Model> misplaced_models_actual_coords_wrt_world;
   vector<osrf_gear::Model> misplaced_models_desired_coords_wrt_world;
   vector<osrf_gear::Model> missing_models_wrt_world;
   vector<osrf_gear::Model> orphan_models_wrt_world;

    ROS_INFO("main: instantiating an object of type BoxInspector");
    BoxInspector boxInspector(&nh);  

    ROS_INFO("setting up a subscriber for an order: ");

    //can queue up as many as 10 orders
    ros::Subscriber order_subscriber = nh.subscribe("/ariac/orders", 10, order_callback);
    ROS_INFO("waiting to receive an order");
    while (!g_order_received) {
      ros::spinOnce();
      ros::Duration(1.0).sleep();
      ROS_INFO("waiting to receive an order");
    }
    ROS_INFO_STREAM("received  order; desired shipment is: "<< g_shipment_wrt_box<<endl);


    //find the pose of the shipment box:
    geometry_msgs::PoseStamped box_pose_wrt_world;
    if (!boxInspector.get_box_pose_wrt_world(box_pose_wrt_world)) {
        ROS_WARN("could  not find box! quitting");
        return 1;
    } 
    ROS_INFO_STREAM("main: found box at box_pose_wrt_world="<<box_pose_wrt_world<<endl);
    //int ans;  //poor-man's break-point
    //cout<<"enter 1: ";
    //cin>>ans;
    
    //convert desired part poses from pose w/rt box to a vector of models w/ poses w/rt world
    //need  to know where the box actually IS in order to convert to desired world-space part coords
    //this step is necessary in order to tell the robot where to place parts (w/rt world)
    ROS_INFO("computing desired part poses w/rt world coords: ");
    boxInspector.compute_shipment_poses_wrt_world(g_shipment_wrt_box, box_pose_wrt_world, desired_models_wrt_world);
 
    osrf_gear::Shipment shipment_status;

    boxInspector.model_poses_wrt_box(shipment_status);    

    //main fnc: invoke the box inspector to check on order status:
    //fill in vectors of correctly placed parts, misplaced parts, desired coords of misplaced parts,
    //missing parts, and extraneous parts
    //INPUT desired_models_wrt_world; fill  in the rest of the vectors using this function call
    boxInspector.update_inspection(desired_models_wrt_world, satisfied_models_wrt_world,
       misplaced_models_actual_coords_wrt_world, misplaced_models_desired_coords_wrt_world,
       missing_models_wrt_world, orphan_models_wrt_world);

    //print out the results:
    int nmodels = satisfied_models_wrt_world.size();
    ROS_INFO("there are %d models in the correct poses; they are:",nmodels);
    for (int i=0;i<nmodels;i++) {
       ROS_INFO_STREAM(satisfied_models_wrt_world[i]<<endl);
    }

    nmodels = misplaced_models_actual_coords_wrt_world.size();
    ROS_INFO("there are %d models in  incorrect poses; actual vs desired coords are:",nmodels);
    for (int i=0;i<nmodels;i++) {
       ROS_INFO_STREAM(misplaced_models_actual_coords_wrt_world[i]<<endl);
       ROS_INFO_STREAM(misplaced_models_desired_coords_wrt_world[i]<<endl<<endl);
    }

    nmodels = missing_models_wrt_world.size();
    ROS_INFO("there are %d models missing; they are:",nmodels);
    for (int i=0;i<nmodels;i++) {
       ROS_INFO_STREAM(missing_models_wrt_world[i]<<endl);
    }

    nmodels = orphan_models_wrt_world.size();
    ROS_INFO("there are %d orphan models (in box, but don't belong); they are:",nmodels);
    for (int i=0;i<nmodels;i++) {
       ROS_INFO_STREAM(orphan_models_wrt_world[i]<<endl);
    }
    
  


    return 0;
} 

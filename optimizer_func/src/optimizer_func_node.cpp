#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"

#include "optimizer_func/optimizer_func.h"

#include <sstream>

osrf_gear::Order current, priority;
ros::Time current_order_recvd, priority_order_recvd;

osrf_gear::Shipment testing_shipment;

char *shipment_names[] = {"no_active_shipment", "order_0_shipment_0", "order_0_shipment_1", "order_1_shipment_0", "order_1_shipment_1"};

int alert_level = LEVEL_GREEN;


// A quick and dirty simplification for creating test messages
#define ADD_PRODUCT(prod, type_string, xp, yp, zp, xo, yo, zo, wo)  {  \
    prod.type.append(type_string);					\
    prod.pose.position.x = xp;						\
    prod.pose.position.y = yp;						\
    prod.pose.position.z = zp;						\
    prod.pose.orientation.x = xo;					\
    prod.pose.orientation.y = yo;					\
    prod.pose.orientation.z = zo;					\
    prod.pose.orientation.w = wo;					\
  }

void clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg) {
  if ((msg->clock.sec > 500) && (alert_level < LEVEL_COMPLETE)) {
    ROS_WARN("Setting alert_level to LEVEL_COMPLETE");
    alert_level = LEVEL_COMPLETE;
  } else if ((msg->clock.sec > 460) && (alert_level < LEVEL_RED)) {
    ROS_WARN("Setting alert_level to LEVEL_RED");
    alert_level = LEVEL_RED;
  } else if ((msg->clock.sec > 420) && (alert_level < LEVEL_YELLOW)) {
    ROS_WARN("Setting alert_level to LEVEL_YELLOW");
    alert_level = LEVEL_YELLOW;
  }
}

// Listening for the Orders from ARIAC
void orderCallback(const osrf_gear::Order::ConstPtr& msg) {

  osrf_gear::Order msg2 = *msg;
  
  ROS_INFO("Received order %s with %i shipment%s", msg->order_id.c_str(), (int) msg->shipments.size(), msg->shipments.size() == 1 ? "": "s");
  ROS_DEBUG("shipment_queue is so long: %i", (int)shipment_queue.shipments.size());
  if (!msg->order_id.compare(0, 6, "order_", 0, 6)) {
    ROS_DEBUG("Identified order: %s as an original order", msg->order_id.c_str());
    // TODO: This could be better.
    if (!msg->order_id.compare("order_0")) {
      current_order_recvd= ros::Time::now();
    } else if (!msg->order_id.compare("order_1")) {
      priority_order_recvd= ros::Time::now();
    }

    // Strip out any exising shipments that this updates.
    for (int indy = shipment_queue.shipments.size()-1; indy >= 0; --indy) {
      for (int indx = msg2.shipments.size()-1; indx >= 0; --indx) {
	// Strip out any previous order_ns
	if (shipping_now.size() > 0) {
	  if (!msg2.shipments[indx].shipment_type.compare(shipping_now[0].shipment_type)) {
	    ROS_INFO("Updating %s in the currently being shipped.", shipment_queue.shipments[indy].shipment_type.c_str());
	    shipping_now.push_back(msg2.shipments[indx]);
	    msg2.shipments.pop_back();
	    break;
	  }
	}
	if (!msg2.shipments[indx].shipment_type.compare(shipment_queue.shipments[indy].shipment_type)) {
	  ROS_INFO("Updating %s", shipment_queue.shipments[indy].shipment_type.c_str());
	  shipment_queue.shipments[indy] = msg2.shipments[indx];
	  msg2.shipments.pop_back();
	  break;
	}
      }
    }
    // Add the (remaining) shipments to the shipment_queue.
    if (!msg2.order_id.compare(0, 7, "order_0", 0, 7)) {
      for (int indx = 0; indx < msg2.shipments.size(); indx++) {
	ROS_INFO("Adding %s to the end of the shipment_queue.", msg2.shipments[indx].shipment_type.c_str());
	shipment_queue.shipments.insert(shipment_queue.shipments.end()-1, msg2.shipments[indx]);
      }
    } else {
      for (int indx = 0; indx < msg2.shipments.size(); indx++) {
	ROS_INFO("Adding %s to the begining of the shipment_queue.", msg2.shipments[indx].shipment_type.c_str());
	shipment_queue.shipments.insert(shipment_queue.shipments.begin() + ((shipment_queue.shipments.size()) > 1 ? 1 : 0), msg2.shipments[indx]);
      }
    }

  } else {
    ROS_ERROR("Order was not identified!!!");
  }
  ROS_DEBUG("shipment_queue is so long: %i", (int)shipment_queue.shipments.size());

  for (int indx = 0; indx < shipment_queue.shipments.size(); indx++) {
    ROS_INFO("Shipment in queue position %i: %s", indx, shipment_queue.shipments[indx].shipment_type.c_str());
  }
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "optimizer");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  // initialize the shipment_queue with a null shipment
  shipment_queue.shipments.resize(1);
  shipment_queue.shipments[0].shipment_type.append(NULL_SHIPMENT);
  
  ros::ServiceServer service = n.advertiseService("optimizer", optimize_shipments);
  ROS_INFO("Ready to respond to optimization requests.");

  ros::Subscriber sub = n.subscribe("ariac/orders", 5, orderCallback);
  ros::Subscriber clk = n.subscribe("clock", 1, clockCallback);

  
  // Below is testing stuff.

  // current.shipments.resize(1);
  // current.shipments[0].shipment_type.append("order_0_shipment_0");
  // current.shipments[0].products.resize(6);

  // ADD_PRODUCT(current.shipments[0].products[0], "gasket_part", 0.1, -0.2, 0.0, 0.0, 0.0, -sqrt(2)/2, sqrt(2)/2);
  // ADD_PRODUCT(current.shipments[0].products[1], "gasket_part", -0.1, -0.2, 0.0, 0.0, 0.0, -sqrt(2)/2, sqrt(2)/2);
  // ADD_PRODUCT(current.shipments[0].products[2], "piston_rod_part", 0.15, 0.15, 0.0, 0.0, 0.0, 0.0, 1.0);
  // ADD_PRODUCT(current.shipments[0].products[3], "piston_rod_part", -0.15, 0.15, 0.0, 0.0, 0.0, 0.0, 1.0);
  // ADD_PRODUCT(current.shipments[0].products[4], "piston_rod_part", 0.0, 0.15, 0.0, 0.0, 0.0, 1.0, 0.0);
  // ADD_PRODUCT(current.shipments[0].products[5], "gear_part", 0.15, -0.1, 0.0, 0.0, 0.0, 0.0, 1.0);
  
  // priority.shipments.resize(2);
  // priority.shipments[0].shipment_type.append("order_1_shipment_0");
  // priority.shipments[0].products.resize(6);
  // priority.shipments[1].shipment_type.append("order_1_shipment_1");
  // priority.shipments[1].products.resize(5);

  // ADD_PRODUCT(priority.shipments[0].products[0], "pulley_part", -0.1, -0.2, 0.0, 1.0, 0.0, 0.0, 0.0);
  // ADD_PRODUCT(priority.shipments[0].products[1], "piston_rod_part", 0.0, 0.15, 0.0, 0.0, 0.0, 1.0, 0.0);
  // ADD_PRODUCT(priority.shipments[0].products[2], "gear_part", 0.15, 0.1, 0.0, 0.0, 0.0, 0.0, 1.0);
  // ADD_PRODUCT(priority.shipments[0].products[3], "gear_part", 0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  // ADD_PRODUCT(priority.shipments[0].products[4], "gear_part", 0.15, -0.2, 0.0, 0.0, 0.0, 0.0, 1.0);
  // ADD_PRODUCT(priority.shipments[0].products[5], "gear_part", 0.15, -0.1, 0.0, 0.0, 0.0, 0.0, 1.0);
  
  // ADD_PRODUCT(priority.shipments[1].products[0], "gasket_part", 0.1, -0.2, 0.0, 0.0, 0.0, -sqrt(2)/2, sqrt(2)/2);
  // ADD_PRODUCT(priority.shipments[1].products[1], "gasket_part", -0.1, -0.2, 0.0, 0.0, 0.0, -sqrt(2)/2, sqrt(2)/2);
  // ADD_PRODUCT(priority.shipments[1].products[2], "piston_rod_part", 0.15, 0.15, 0.0, 0.0, 0.0, 0.0, 1.0);
  // ADD_PRODUCT(priority.shipments[1].products[3], "piston_rod_part", -0.15, 0.15, 0.0, 0.0, 0.0, 0.0, 1.0);
  // ADD_PRODUCT(priority.shipments[1].products[4], "piston_rod_part", 0.0, 0.15, 0.0, 0.0, 0.0, 1.0, 0.0);

  // osrf_gear::Shipment loaded_parts;
  // loaded_parts.products.resize(3);

  // loaded_parts.products[0] = current.shipments[0].products[1];
  // loaded_parts.products[1] = current.shipments[0].products[2];
  // loaded_parts.products[2] = current.shipments[0].products[5];
  
  // decision_func(loaded_parts, ros::Duration(0.0));

  // optimizer_func::optimizer_msgs::Request req;
  // optimizer_func::optimizer_msgs::Response res;
  // osrf_gear::Shipment empty_shipment;
  
  // req.loaded = loaded_parts;
  // req.orphaned = empty_shipment;
  // req.reposition = empty_shipment;
  // req.missing = empty_shipment;

  //std::cout<<"enter 1 to continue: ";
  //int ans;
  //std::cin>>ans;

  // current_order_recvd = ros::Time::now(); // - ros::Duration(135.0);
  // priority_order_recvd = ros::Time::now();
  
  // req.giving_up = optimizer_func::optimizer_msgs::Request::NOT_GIVING_UP;

  // Run the optimization routine
  // optimize_shipments(req, res);

  // Act like a proper service
  ros::spin();
  
  return 0;
}

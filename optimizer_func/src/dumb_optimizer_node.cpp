#include "ros/ros.h"
#include "optimizer_func/optimizer_msgs.h"

#include "osrf_gear/Shipment.h"
#include "osrf_gear/Order.h"

osrf_gear::Order current, priority;
ros::Time current_order_recvd, priority_order_recvd;

char *shipment_names[] = {"no_active_shipment", "order_0_shipment_0", "order_0_shipment_1", "order_1_shipment_0", "order_1_shipment_1"};

int case_number = 0;

bool optimize_shipments(optimizer_func::optimizer_msgs::Request  &req,
			optimizer_func::optimizer_msgs::Response &res)
{
  osrf_gear::Shipment empty_shipment;
  
  if (((req.loaded.products.size() > 0) &&		\
       (req.orphaned.products.size() == 0) &&		\
       (req.missing.products.size() == 0) &&		\
       (req.reposition.products.size() == 0)) ||	\
      (req.giving_up == optimizer_func::optimizer_msgsRequest::GIVING_UP)) {

    switch (case_number) {
    case 0:
      break;
    case 1:
      if (current.shipments.size() > 1) {
	case_number = 2;
	res.shipment = current.shipments[1];
	
      } else {
	case_number = 3;
	res.shipment = priority.shipments[0];
      }
      break;
    case 2:
      case_number = 3;
      res.shipment = priority.shipments[0];
      break;
    case 3:
      if (priority.shipments.size() > 1) {
	case_number = 4;
	res.shipment = priority.shipments[1];
      }
      break;
    case 4:
      case_number = 5;
      res.shipment = empty_shipment;
      break;
    }	
    
    res.decision = optimizer_func::optimizer_msgsResponse::PRIORITY_LOAD_NEXT;
  } else {
  
    switch (case_number) {
    case 0:
      if (current.shipments.size() > 0) {
	case_number = 1;
	res.shipment = current.shipments[0];
      } else {
	res.shipment = empty_shipment;
      }
      break;
    case 1:
      res.shipment = current.shipments[0];
      break;
    case 2:
      res.shipment = current.shipments[1];
      break;
    case 3:
      res.shipment = priority.shipments[0];
      break;
    case 4:
      res.shipment = priority.shipments[1];
      break;
    default:
      res.shipment = empty_shipment;
    }
    res.decision = optimizer_func::optimizer_msgsResponse::USE_CURRENT_BOX;
  }
  return true;
}

void orderCallback(const osrf_gear::Order::ConstPtr& msg) {

  if (! msg->order_id.compare("order_0")) {
    current_order_recvd.now();
    current = *msg;
  } else if (!msg->order_id.compare("order_0_update_0")) {
    current = *msg;
  } else if (!msg->order_id.compare("order_1")) {
    priority = *msg;
  } else if (!msg->order_id.compare("order_1_update_0")) {
    priority = *msg;
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dumb_optimizer");
  ros::NodeHandle n;

  current.shipments.resize(1);
  current.shipments[0].shipment_type.append(shipment_names[0]);
  
  ros::ServiceServer service = n.advertiseService("optimizer", optimize_shipments);
  ROS_INFO("Ready to respond (dumbly) to optimization requests.");

  ros::Subscriber sub = n.subscribe("ariac/orders", 10, orderCallback);
  
  ros::spin();

  return 0;
}

#include "optimizer_func/optimizer_func.h"
#include "optimizer_func/helper_funcs.h"


bool optimize_shipments(optimizer_func::optimizer_msgs::Request  &req,
			optimizer_func::optimizer_msgs::Response &res)
{
  osrf_gear::Shipment empty_shipment;
  empty_shipment.shipment_type.append("no_shipment");

  if (req.inspection_site == optimizer_func::optimizer_msgsRequest::Q1_STATION) {
    ROS_INFO("Request from Q1");
    if (req.giving_up == optimizer_func::optimizer_msgsRequest::GIVING_UP) {
      ROS_INFO("Giving up set, so advance the box (next shipment returned).");
      res.decision = optimizer_func::optimizer_msgsResponse::ADVANCE_THIS_BOX_TO_Q2;
      if (!shipment_queue.shipments[0].shipment_type.compare(req.loaded.shipment_type)) {
	if(shipment_queue.shipments.size() > 1) 
	  shipment_queue.shipments.erase(shipment_queue.shipments.begin());
      }	else
	ROS_WARN("Expecting shipment %s, and received shipment %s", shipment_queue.shipments[0].shipment_type.c_str(), req.loaded.shipment_type.c_str());
    } else {
      ROS_INFO("Not giving up, so keep using the same box (same shipment returned).");
      res.decision = optimizer_func::optimizer_msgsResponse::USE_CURRENT_BOX;
    }
    ROS_INFO("Sending next shipment; queue size: %i", (int) shipment_queue.shipments.size());
  } else if (req.inspection_site == optimizer_func::optimizer_msgsRequest::Q2_STATION) {
    ROS_INFO("Request from Q2");
    ROS_INFO("Always advancing box (showing next shipment).");
    res.decision = optimizer_func::optimizer_msgsResponse::PRIORITY_LOAD_NEXT;
  } else {
    ROS_ERROR("Request did not come from Q1 or Q2!!");
    res.decision = optimizer_func::optimizer_msgsResponse::USE_CURRENT_BOX;
  }
  ROS_INFO("Sending information on %s", shipment_queue.shipments[0].shipment_type.c_str());
  res.shipment = shipment_queue.shipments[0];

  return true;
}


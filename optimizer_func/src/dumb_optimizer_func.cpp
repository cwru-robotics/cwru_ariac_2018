#include "optimizer_func/optimizer_func.h"
#include "optimizer_func/helper_funcs.h"

int short_term_memory = -1;
int middle_term_memory = 0;

// Could make this a time based decision instead of being based on count
#define NUM_REP_REQS	8

bool optimize_shipments(optimizer_func::optimizer_msgs::Request  &req,
			optimizer_func::optimizer_msgs::Response &res)
{


  ROS_INFO("Asking about %s from Q%i, have %s as next in queue", req.loaded.shipment_type.c_str(), req.inspection_site, shipment_queue.shipments[0].shipment_type.c_str());
  if (req.inspection_site == optimizer_func::optimizer_msgsRequest::Q1_STATION) {
    ROS_INFO("Request from Q1");
    if ((req.giving_up == optimizer_func::optimizer_msgsRequest::GIVING_UP) || (middle_term_memory > NUM_REP_REQS)) {
      ROS_INFO("Giving up set, so advance the box (next shipment returned).");
      short_term_memory = -1;
      res.decision = optimizer_func::optimizer_msgsResponse::ADVANCE_THIS_BOX_TO_Q2;
      if ((!shipment_queue.shipments[0].shipment_type.compare(req.loaded.shipment_type)) || (!req.loaded.shipment_type.compare("no_active_shipment"))) {
	if (!req.loaded.shipment_type.compare("no_active_shipment")) {
	  ROS_WARN("The optimizer expected %s, but received %s instead", shipment_queue.shipments[0].shipment_type.c_str(), req.loaded.shipment_type.c_str());
	}
	if(shipment_queue.shipments.size() > 1) 
	  shipment_queue.shipments.erase(shipment_queue.shipments.begin());
      }	else
	ROS_WARN("Expecting shipment %s, and received shipment %s", shipment_queue.shipments[0].shipment_type.c_str(), req.loaded.shipment_type.c_str());
    } else {
      ROS_INFO("Not giving up, so keep using the same box (same shipment returned).");
      if (short_term_memory == req.loaded.products.size() + req.orphaned.products.size() + req.missing.products.size() + req.reposition.products.size()) {
	middle_term_memory++;
	ROS_INFO("middle_term_memory: %i", middle_term_memory);
      } else
	middle_term_memory = 0;
      short_term_memory = req.loaded.products.size() + req.orphaned.products.size() + req.missing.products.size() + req.reposition.products.size();
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


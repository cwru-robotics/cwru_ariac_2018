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
      shipment_queue_indx++;
    } else {
      res.decision = optimizer_func::optimizer_msgsResponse::USE_CURRENT_BOX;
      ROS_INFO("Not giving up, so keep using the same box (same shipment returned).");
    }
    if(shipment_queue.shipments.size() > shipment_queue_indx) {
      ROS_INFO("Sending next shipment; queue size %i: indx: %i", (int) shipment_queue.shipments.size(), shipment_queue_indx);
      res.shipment = shipment_queue.shipments[shipment_queue_indx];
    } else {
      ROS_INFO("Sending empty shipment; queue size %i: indx: %i", (int) shipment_queue.shipments.size(), shipment_queue_indx);
      res.shipment = empty_shipment;
    } 
  } else if(req.inspection_site == optimizer_func::optimizer_msgsRequest::Q2_STATION) {
    ROS_INFO("Request from Q2");
    ROS_INFO("Always advancing box (showing next shipment).");
    res.decision = optimizer_func::optimizer_msgsResponse::PRIORITY_LOAD_NEXT;
    // if (req.giving_up == optimizer_func::optimizer_msgsRequest::GIVING_UP) {
    // if(shipment_queue.shipments.size() > shipment_queue_indx) {
    //   ROS_INFO("Sending next shipment; queue size %i: indx: %i", (int) shipment_queue.shipments.size(), shipment_queue_indx);
    //   res.shipment = shipment_queue.shipments[shipment_queue_indx];
    // } else {
    //   ROS_INFO("Sending empty shipment; queue size %i: indx: %i", (int) shipment_queue.shipments.size(), shipment_queue_indx);
    //   res.shipment = empty_shipment;
    // } 
    // }
  } else {
    ROS_ERROR("Request did not come from Q1 or Q2!!");
  }

  return true;
}


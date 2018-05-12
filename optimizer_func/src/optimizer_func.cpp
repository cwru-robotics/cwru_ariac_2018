#include "optimizer_func/optimizer_func.h"
#include "optimizer_func/helper_funcs.h"
#include "optimizer_func/decider.h"

int current_shipment = 0;

int short_term_memory_Q1 = 0;
int middle_term_memory_Q1 = 0;

int short_term_memory_Q2 = 0;
int middle_term_memory_Q2 = 0;

std::vector <osrf_gear::Shipment> shipping_now;

// Could make this a time based decision instead of being based on count
#define NUM_REP_REQS	8

bool optimize_shipments(optimizer_func::optimizer_msgs::Request  &req,
			optimizer_func::optimizer_msgs::Response &res)
{

  ROS_INFO("Q%i asking about %s.  Currently queued is %s.", req.inspection_site, req.loaded.shipment_type.c_str(), shipment_queue.shipments[0].shipment_type.c_str());

  // Do different things based on which station, Q1 or Q1, are querying.
  switch (req.inspection_site) {
  case optimizer_func::optimizer_msgsRequest::Q1_STATION:
    
    // Giving up or there were a lot of repeats without indication of moving forward
    if ((req.giving_up == optimizer_func::optimizer_msgsRequest::GIVING_UP) || (middle_term_memory_Q1 >= NUM_REP_REQS) || (alert_level == LEVEL_RED) || ((alert_level == LEVEL_YELLOW) && (shipping_now.size() > 0))) {
      
      ROS_INFO("Giving up set, so advance the box (next shipment returned).");
      
      // Set the decision to move forward as requested by the "give_up" flag.
      res.decision = optimizer_func::optimizer_msgsResponse::ADVANCE_THIS_BOX_TO_Q2;
      
      // Make sure that the shipment query is for the expected shipment.  Also accepting a NULL_SHIPMENT string
      if (shipment_queue.shipments[0].shipment_type == req.loaded.shipment_type) {
	// The last entry is a empty shipment queue placeholder, so don't pop it.
	if(shipment_queue.shipments.size() > 1) 
	  shipment_queue.shipments.erase(shipment_queue.shipments.begin());
      }	else
	ROS_WARN("Expecting shipment %s, and received shipment %s", shipment_queue.shipments[0].shipment_type.c_str(), req.loaded.shipment_type.c_str());
      
      // Reset middle term memory.
      middle_term_memory_Q1 = 0;
      
    } else if (req.giving_up == optimizer_func::optimizer_msgsRequest::NOT_GIVING_UP) {
      ROS_INFO("Not giving up, so keep using the same box (same shipment returned).");
      
      // Make sure that the shipment query is for the expected shipment.  
      if (shipment_queue.shipments[0].shipment_type == std::string(req.loaded.shipment_type)) {
	// Short term memory keeps track of the state of the shipment process 
	// to prevent a no-progress state from continuing too long.
	if (short_term_memory_Q1 == req.loaded.products.size() + req.orphaned.products.size() + req.missing.products.size() + req.reposition.products.size()) {
	  // Increment a middle term memory which is checked to force a "give_up" above when reaching NUM_REP_REQS.
	  middle_term_memory_Q1++;
	  ROS_WARN("middle_term_memory_Q1: %i", middle_term_memory_Q1);
	} else {
	  // Reset the middle term memory 
	  middle_term_memory_Q1 = 0;
	}
	
	// Set the short term memory.
	short_term_memory_Q1 = req.loaded.products.size() + req.orphaned.products.size() + req.missing.products.size() + req.reposition.products.size();
	
	// This is where a decision is made about how to proceed.
	// The decider function returns zero always in the dumb_optimizer.
	if (deciderQ1(req, res)) {
	  res.decision = optimizer_func::optimizer_msgsResponse::ADVANCE_THIS_BOX_TO_Q2;
	  // The last entry is a empty shipment queue placeholder, so don't pop it.
	  if(shipment_queue.shipments.size() > 1) {
	    // Save what is now being shipped.
	    shipping_now.push_back(shipment_queue.shipments[0]);
	    shipment_queue.shipments.erase(shipment_queue.shipments.begin());
	  } else {
	    res.decision = optimizer_func::optimizer_msgsResponse::USE_CURRENT_BOX;
	  }
	  ROS_INFO("Sending next shipment; queue size: %i", (int) shipment_queue.shipments.size());
	}
      }
    } else {
      ROS_ERROR("Not a known giving_up entry!! Just advancing shipment.");
      res.decision = optimizer_func::optimizer_msgsResponse::ADVANCE_THIS_BOX_TO_Q2;
    }
    
    break;
    
    // Section for Q2 queries
  case optimizer_func::optimizer_msgsRequest::Q2_STATION:
    
    // Giving up
    if ((req.giving_up ==  optimizer_func::optimizer_msgsRequest::GIVING_UP) || (middle_term_memory_Q2 >= NUM_REP_REQS) || (alert_level == LEVEL_RED) || ((alert_level == LEVEL_YELLOW) && (shipping_now.size() > 0))) {
      ROS_INFO("Giving up.  Responding PRIORIYT_LOAD_NEXT and work on shipment %s", shipment_queue.shipments[0].shipment_type.c_str());
      res.decision = optimizer_func::optimizer_msgsResponse::PRIORITY_LOAD_NEXT;
      middle_term_memory_Q2 = 0;
    } else if (req.giving_up ==  optimizer_func::optimizer_msgsRequest::NOT_GIVING_UP) {
      ROS_ERROR("Not giving_up at Q2.");
      
      // Make sure that the shipment query is for the expected shipment.  Also accepting a NULL_SHIPMENT string
      if (shipping_now[0].shipment_type == std::string(req.loaded.shipment_type)) {
	// TODO:
	// The only thing being checked here is whether the overall score will be substanially
	// dropped by not removing a faulty part.
	
	// Short term memory keeps track of the state of the shipment process 
	// to prevent a no-progress state from continuing too long.
	if (short_term_memory_Q2 == req.loaded.products.size() + req.orphaned.products.size() + req.missing.products.size() + req.reposition.products.size()) {
	  // Increment a middle term memory which is checked to force a "give_up" above when reaching NUM_REP_REQS.
	  middle_term_memory_Q2++;
	  ROS_WARN("middle_term_memory_Q2: %i", middle_term_memory_Q2);
	} else {
	  // Reset the middle term memory 
	  middle_term_memory_Q2 = 0;
	}
	// Set the short term memory.
	short_term_memory_Q2 = req.loaded.products.size() + req.orphaned.products.size() + req.missing.products.size() + req.reposition.products.size();
	
	// If there isn't anything missing, orphaned, or in need of repositioning, or if the alert_level is now red, send the box on its way.
	if (((req.missing.products.size() == 0) && (req.orphaned.products.size() == 0) && (req.reposition.products.size() == 0)) || (alert_level == LEVEL_RED)) {
	  ROS_INFO("Not giving up, check to decide whether to do anything.");
	  res.decision = optimizer_func::optimizer_msgsResponse::PRIORITY_LOAD_NEXT;
	} else {
	  
	  // Make a decision about whether to replace a faulty part.

	  // If the shipment was updated, drop the old one.
	  if (shipping_now.size() > 1)
	    shipping_now.erase(shipping_now.begin());
	  
	  // Default decision is to send it on its way for now, but here is where the decision can be made.
	  if (deciderQ2(req, res)) {
	    res.decision = optimizer_func::optimizer_msgsResponse::USE_CURRENT_BOX;
	    // Need to send the correct shipment_type back

	    if (shipping_now.size() > 0)
	      res.shipment = shipping_now[0];
	    else
	      res.shipment = req.loaded;
	    
	    //Must return here so that shipment is correct.
	    return true;
	    
	  } else {
	    res.decision = optimizer_func::optimizer_msgsResponse::PRIORITY_LOAD_NEXT;
	  }
	}
      }
    } else {
      ROS_ERROR("Not a known giving_up entry!! Just advancing shipment.");
      res.decision = optimizer_func::optimizer_msgsResponse::PRIORITY_LOAD_NEXT;
    }
    break;
    
  default:
    ROS_ERROR("Request did not come from Q1 or Q2!!");
    res.decision = optimizer_func::optimizer_msgsResponse::USE_CURRENT_BOX;
    break;
  }
  ROS_INFO("Sending information on %s", shipment_queue.shipments[0].shipment_type.c_str());
  res.shipment = shipment_queue.shipments[0];

  return true;
}

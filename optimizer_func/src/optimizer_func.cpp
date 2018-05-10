#include "optimizer_func/optimizer_func.h"
#include "optimizer_func/helper_funcs.h"


int current_shipment = 0;

int short_term_memory = -1;
int middle_term_memory = 0;

// Could make this a time based decision instead of being based on count
#define NUM_REP_REQS	8

bool optimize_shipments(optimizer_func::optimizer_msgs::Request  &req,
			optimizer_func::optimizer_msgs::Response &res) {


  ROS_INFO("Asking about %s from Q%i, have %s as next in queue", req.loaded.shipment_type.c_str(), req.inspection_site, shipment_queue.shipments[0].shipment_type.c_str());
  if (req.inspection_site == optimizer_func::optimizer_msgsRequest::Q1_STATION) {
    ROS_INFO("Request from Q1");
    if ((req.giving_up == optimizer_func::optimizer_msgsRequest::GIVING_UP) || (middle_term_memory > NUM_REP_REQS)) {
      ROS_INFO("Giving up set, so advance the box (next shipment returned).");
      short_term_memory = -1;
      res.decision = optimizer_func::optimizer_msgsResponse::ADVANCE_THIS_BOX_TO_Q2;
      if ((!shipment_queue.shipments[0].shipment_type.compare(req.loaded.shipment_type)) || (!req.loaded.shipment_type.compare(NULL_SHIPMENT))) {
	if (!req.loaded.shipment_type.compare(NULL_SHIPMENT)) {
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
    if (req.giving_up ==  optimizer_func::optimizer_msgsRequest::GIVING_UP) {
      ROS_INFO("Giving up.  Responding PRIORIYT_LOAD_NEXT and work on shipment %s", shipment_queue.shipments[0].shipment_type.c_str());
      res.decision = optimizer_func::optimizer_msgsResponse::PRIORITY_LOAD_NEXT;
    } else if (req.giving_up ==  optimizer_func::optimizer_msgsRequest::NOT_GIVING_UP) {
      // TODO:
      // The only thing being checked here is whether the overall score will be substanially
      // dropped by not removing a faulty part.
      ROS_INFO("Not giving up, check to decide whether to do anything.");
      res.decision = optimizer_func::optimizer_msgsResponse::USE_CURRENT_BOX;
    } else {
      ROS_ERROR("Error state!! Just advancing shipment.");
      res.decision = optimizer_func::optimizer_msgsResponse::PRIORITY_LOAD_NEXT;
    }
  } else {
    ROS_ERROR("Request did not come from Q1 or Q2!!");
    res.decision = optimizer_func::optimizer_msgsResponse::USE_CURRENT_BOX;
  }
  ROS_INFO("Sending information on %s", shipment_queue.shipments[0].shipment_type.c_str());
  res.shipment = shipment_queue.shipments[0];
  ROS_INFO("Asking about %s from Q%i, have %s as next in queue", req.loaded.shipment_type.c_str(), req.inspection_site, shipment_queue.shipments[0].shipment_type.c_str());
  if (req.inspection_site == optimizer_func::optimizer_msgsRequest::Q1_STATION) {
    ROS_INFO("Request from Q1");
    if (req.giving_up == optimizer_func::optimizer_msgsRequest::GIVING_UP) {
      ROS_INFO("Giving up set, so advance the box (next shipment returned).");
      res.decision = optimizer_func::optimizer_msgsResponse::ADVANCE_THIS_BOX_TO_Q2;
      if ((!shipment_queue.shipments[0].shipment_type.compare(req.loaded.shipment_type)) || (!req.loaded.shipment_type.compare(NULL_SHIPMENT))) {
	if(shipment_queue.shipments.size() > 1) 
	  shipment_queue.shipments.erase(shipment_queue.shipments.begin());
      }	else
	ROS_WARN("Expecting shipment %s, and received shipment %s", shipment_queue.shipments[0].shipment_type.c_str(), req.loaded.shipment_type.c_str());
    } else {
      ROS_INFO("Not giving up, so keep using the same box (same shipment returned).");

      // TODO: 
      // Peform optimization here.  Results in altering the shipment_queue.
      // Should only do this if there is an "order_1."
      // Should only change the order of the queue if there is more than a modest benefit.
      
      res.decision = optimizer_func::optimizer_msgsResponse::USE_CURRENT_BOX;
    }
    ROS_INFO("Sending next shipment; queue size: %i", (int) shipment_queue.shipments.size());
  } else if (req.inspection_site == optimizer_func::optimizer_msgsRequest::Q2_STATION) {
    ROS_INFO("Request from Q2");
    if (req.giving_up ==  optimizer_func::optimizer_msgsRequest::GIVING_UP) {
      ROS_INFO("Giving up.  Responding PRIORIYT_LOAD_NEXT and work on shipment %s", shipment_queue.shipments[0].shipment_type.c_str());
      res.decision = optimizer_func::optimizer_msgsResponse::PRIORITY_LOAD_NEXT;
    } else if (req.giving_up ==  optimizer_func::optimizer_msgsRequest::NOT_GIVING_UP) {
      // TODO:
      // The only thing being checked here is whether the overall score will be substanially
      // dropped by not removing a faulty part.
      ROS_INFO("Not giving up, check to decide whether to do anything.");
      res.decision = optimizer_func::optimizer_msgsResponse::USE_CURRENT_BOX;
    } else {
      ROS_ERROR("Error state!! Just advancing shipment.");
      res.decision = optimizer_func::optimizer_msgsResponse::PRIORITY_LOAD_NEXT;
    }
  } else {
    ROS_ERROR("Request did not come from Q1 or Q2!!");
    res.decision = optimizer_func::optimizer_msgsResponse::PRIORITY_LOAD_NEXT;

  }
  res.shipment = shipment_queue.shipments[0];

  return true;

  ROS_INFO("Beginning optimization for %s", req.loaded.shipment_type.c_str());

  // Count the total parts being ordered. Not doing anything with the infor now.
  int part_counts[the_parts_count];
  memset(&part_counts, 0x00, sizeof(part_counts));
  
  count_parts(current, priority, (int *) &part_counts);

  // Set up an empty shipment for the time being
  osrf_gear::Shipment null_shipment;
  
  // ROS_DEBUG("Part names:    %s %s %s %s %s", the_parts[0], the_parts[1], the_parts[2], the_parts[3], the_parts[4]);
  // ROS_DEBUG("%i\t%i\t%i\t%i\t%i", part_counts[0], part_counts[1], part_counts[2], part_counts[3], part_counts[4]);

  // If there is not an order yet, just send an empty response
  // if ((current.shipments.size() < 1) && (priority.shipments.size() < 1)) {
  if ((req.inspection_site == optimizer_func::optimizer_msgsRequest::Q1_STATION)) {
    ROS_INFO("No orders in the queue.");
    if (req.giving_up == optimizer_func::optimizer_msgsRequest::GIVING_UP) {
      res.decision = optimizer_func::optimizer_msgsResponse::ADVANCE_THIS_BOX_TO_Q2;
    } else {
      res.decision = optimizer_func::optimizer_msgsResponse::USE_CURRENT_BOX;
    }
    res.shipment = shipment_queue.shipments[0];
    return true;
  }

  if (req.inspection_site == optimizer_func::optimizer_msgsRequest::Q2_STATION) {
    if (req.giving_up == optimizer_func::optimizer_msgsRequest::GIVING_UP) {
      res.decision = optimizer_func::optimizer_msgsResponse::PRIORITY_LOAD_NEXT;
      if (shipment_queue.shipments.size() < 1) {
	res.shipment = null_shipment;
      } else {
	res.shipment = shipment_queue.shipments[0];
      }
    } else {
      // This is a condition to complete
      // res.decision = optimizer_func::optimizer_msgsResponse::USE_CURRENT_BOX;
      res.decision = optimizer_func::optimizer_msgsResponse::PRIORITY_LOAD_NEXT;
    }
    return true;
  }


  // If the current shipment if full, or giving_up, advance it
  // if ((current.shipments[0].products.size() > 0) && ((req.loaded.products.size() == current.shipments[0].products.size()) || (req.giving_up == optimizer_func::optimizer_msgsRequest::GIVING_UP)) ) {
  if ((req.inspection_site == optimizer_func::optimizer_msgsRequest::Q1_STATION) && (shipment_queue.shipments[0].products.size() > 0) && ((req.loaded.products.size() == shipment_queue.shipments[0].products.size()) || (req.giving_up == optimizer_func::optimizer_msgsRequest::GIVING_UP)) ) {
    if (req.loaded.products.size() == current.shipments[0].products.size()) {
      ROS_INFO("Shipment is full, move it along");
    }
    if (req.giving_up == optimizer_func::optimizer_msgsRequest::GIVING_UP) {
      ROS_INFO("Giving up on shipment, move it along");
    }
    res.decision = optimizer_func::optimizer_msgsResponse::ADVANCE_THIS_BOX_TO_Q2;

    // If there is another shipment in the order, start on that one, otherwise 
    if (priority.shipments.size() > 0)
      res.shipment = priority.shipments[0];
    else if (current.shipments.size() > 1 )
      res.shipment = current.shipments[1];
    else
      res.shipment = null_shipment;
    return true;
  }
  
  // If there is not a priority order yet, stick with the current order.
  if (priority.shipments.size() < 1) {
    res.decision = optimizer_func::optimizer_msgsResponse::USE_CURRENT_BOX;
    res.shipment = current.shipments[0];
    return true;
  }

  // If the current Order is complete, finish the priority order.
  if ((current.shipments.size() < 1) && (priority.shipments.size() > 0)) {
    res.decision = optimizer_func::optimizer_msgsResponse::USE_CURRENT_BOX;
    res.shipment = priority.shipments[0];
    return true;
  }

  if (req.giving_up == optimizer_func::optimizer_msgsRequest::GIVING_UP) {
    ROS_INFO("The answer is ship the box now");
    res.decision = optimizer_func::optimizer_msgsResponse::ADVANCE_THIS_BOX_TO_Q2;
    if (priority.shipments.size() > 0) {
      res.shipment = priority.shipments[0];
    } else if (current.shipments.size() > 0) {
      res.shipment = current.shipments[0];
    }
    return true;
  }

  // What is loaded in the form of a shipment message
  osrf_gear::Shipment loaded_parts, cur_loaded, cur_in_place, cur_to_load, cur_to_remove, cur_to_move;

  // How does the current shipment look
  ROS_INFO("Compare current shipment to itself");

  // What is in the box.  Lumping it together so that it can be compared to multiple shipments.
  for (int indx = 0; indx < req.loaded.products.size(); indx++) {
    loaded_parts.products.push_back(req.loaded.products[indx]);
  }
  for (int indx = 0; indx < req.orphaned.products.size(); indx++) {
    loaded_parts.products.push_back(req.orphaned.products[indx]);
  }
  for (int indx = 0; indx < req.reposition.products.size(); indx++) {
    loaded_parts.products.push_back(req.reposition.products[indx]);
  }

  ROS_INFO("Starting Comparision");
  // Something like this should be done when choosing which shipment to load first.
  order_breakdown(&(current.shipments[current_shipment]), &loaded_parts, &(current.shipments[current_shipment]), &cur_in_place, &cur_to_load, &cur_to_remove, &cur_to_move);
  
  ROS_INFO("Comparision to priority 0");
  // How does the current shipment compare to the priority order shipment 0
  osrf_gear::Shipment pri0_loaded, pri0_in_place, pri0_to_load, pri0_to_remove, pri0_to_move;
  // ROS_INFO("Compare current shipment to priority order shipment 0");
  order_breakdown(&(current.shipments[current_shipment]), &loaded_parts, &(priority.shipments[0]), &pri0_in_place, &pri0_to_load, &pri0_to_remove, &pri0_to_move);

  ROS_INFO("Comparision to priority 1");
  // How does the corrent shipment compare to the priority order shipment 1
  osrf_gear::Shipment pri1_loaded, pri1_in_place, pri1_to_load, pri1_to_remove, pri1_to_move;
  // ROS_INFO("Compare current shipment to priority order shipment 1");
  order_breakdown(&(current.shipments[current_shipment]), &loaded_parts, &(priority.shipments[1]), &pri1_in_place, &pri1_to_load, &pri1_to_remove, &pri1_to_move);

  ROS_INFO("Starting Timings");

  // What are the timings for different actions
  double cur_cont_timing = 0.0, cur_comp_timing = 0.0;
  double cur_other_comp_timing = 0.0;

  // ROS_INFO("----- Continue loading current order current shipment ------");
  cur_cont_timing = timing_breakdown(cur_to_load, cur_to_remove, cur_to_move);
  // ROS_INFO("----- Complete loading current current shipment ------");
  cur_comp_timing = timing_breakdown(current.shipments[current_shipment], null_shipment, null_shipment);
  if (current.shipments.size() > 1) {
    // ROS_INFO("----- Complete loading current order other shipment ------");
    cur_other_comp_timing = timing_breakdown(current.shipments[!current_shipment], null_shipment, null_shipment);
  }

  // What are the timings for the different actions
  double pri_comp_timing[] = {0.0, 0.0}, pri_swap_timing[] = {0.0, 0.0};
  ROS_INFO("----- Swap loading priority shipment 0 ------");
  pri_swap_timing[0] = timing_breakdown(pri0_to_load, pri0_to_remove, pri0_to_move);
  ROS_INFO("----- Complete loading priority shipment 0 ------");
  pri_comp_timing[0] = timing_breakdown((priority.shipments[0]), null_shipment, null_shipment);

  if (priority.shipments.size() > 1) {
    ROS_INFO("----- Swap loading priority shipment 1 ------");
    pri_swap_timing[1] = timing_breakdown(pri1_to_load, pri1_to_remove, pri1_to_move);
    ROS_INFO("----- Complete loading priority shipment 1 ------");
    pri_comp_timing[1] = timing_breakdown((priority.shipments[1]), null_shipment, null_shipment);
  }
  
  // Which to compare
  int pri_shipment_num = 0;
  if ((priority.shipments.size() > 1) && (pri_swap_timing[1] < pri_swap_timing[0])) {
    pri_shipment_num = 1;
  }
  ROS_INFO(" ****** Comparing current order shipment to priority order shipment %i ******", pri_shipment_num);
  
  // Calculate the TS (Total Scores)

  double cf = 1700.0 / 2100.0;
  double current_order_timings_us[] = {0.0, 0.0, 0.0};
  double current_order_timings_them[] = {0.0, 0.0, 0.0};
  double priority_order_timings[] = {0.0, 0.0, 0.0};

  double cur_elapsed_time = (ros::Time::now().toSec() - current_order_recvd.toSec());
  double pri_elapsed_time = (ros::Time::now().toSec() - priority_order_recvd.toSec());

  // Continue on current shipment
  current_order_timings_us[0] = cur_elapsed_time + cur_cont_timing + cur_other_comp_timing + CONVEYOR_TIME;
  current_order_timings_them[0] = cur_comp_timing + cur_other_comp_timing + CONVEYOR_TIME;
  ROS_INFO("current_order_timings_us[0]: %4.2f _them[0]: %4.2f", current_order_timings_us[0], current_order_timings_them[0]);

  // Convert box to another shipment and switch to priority
  current_order_timings_us[1] = cur_elapsed_time + pri_swap_timing[pri_shipment_num] + cur_comp_timing + cur_other_comp_timing + CONVEYOR_TIME;
  current_order_timings_them[1] = cur_elapsed_time + pri_swap_timing[pri_shipment_num] + cur_comp_timing + cur_other_comp_timing + CONVEYOR_TIME;

  // Ship box incomplete and move to priority
  current_order_timings_us[2] = cur_elapsed_time + pri_comp_timing[0] + pri_comp_timing[1] + cur_comp_timing + cur_other_comp_timing + CONVEYOR_TIME;
  current_order_timings_them[2] = cur_elapsed_time + pri_comp_timing[0] + pri_comp_timing[1] + cur_comp_timing + cur_other_comp_timing + CONVEYOR_TIME;

  // Continue on current shipment
  priority_order_timings[0] = pri_elapsed_time + pri_comp_timing[0] + pri_comp_timing[1] + cur_cont_timing + CONVEYOR_TIME;

  // Convert box to another shipment and switch to priority
  priority_order_timings[1] = pri_elapsed_time + pri_swap_timing[pri_shipment_num] + pri_swap_timing[!pri_shipment_num] + CONVEYOR_TIME;

  // Ship box incomplete and move to priority
  priority_order_timings[2] = pri_elapsed_time + pri_comp_timing[0] + pri_comp_timing[1] + CONVEYOR_TIME;


  double total_score[3][3];
  memset(&total_score, 0x00, sizeof(total_score));
  double completion_score[3];

  // Calculate the completion scores
  completion_score[0] = (2 + PLCMT) * (double) current.shipments[0].products.size() + ((current.shipments.size() > 1) ? (double) current.shipments[1].products.size() : 0.0);
  completion_score[1] = PRIORITY_h * (2 + PLCMT) * (double) priority.shipments[0].products.size() + ((priority.shipments.size() > 1) ? (double) priority.shipments[1].products.size() : 0.0);
  completion_score[2] = (completion_score[0] + completion_score[1]) / 2.0;

  for (int row = 0; row < sizeof(priority_order_timings)/sizeof(priority_order_timings[0]); row++) {
    for (int col = 0; col < sizeof(priority_order_timings)/sizeof(priority_order_timings[0]); col++) {
      double tempa = cf * completion_score[2];
      double tempb = (current_order_timings_us[col] / current_order_timings_them[row]) * completion_score[0];
      double tempc = (priority_order_timings[col] / priority_order_timings[row]) * completion_score[1];

      total_score[row][col] =  tempa +  tempb + tempc;
    }
  }
  ROS_INFO("%4.2f\t%4.2f\t%4.2f", total_score[0][0], total_score[0][1], total_score[0][2]);
  ROS_INFO("%4.2f\t%4.2f\t%4.2f", total_score[1][0], total_score[1][1], total_score[1][2]);
  ROS_INFO("%4.2f\t%4.2f\t%4.2f", total_score[2][0], total_score[2][1], total_score[2][2]);


  double avg_score[] = {(total_score[1][0] + total_score[2][0]) / 2.0, (total_score[0][1] + total_score[2][1]) / 2.0, (total_score[0][2] + total_score[1][2])/ 2.0};
  
  ROS_INFO("Decision Time...");
  ROS_INFO("%2.4f\t%2.4f\t%2.4f", avg_score[0], avg_score[1], avg_score[2]);

  // Need to get decision better.
   if ((avg_score[2] > avg_score[1]) && (avg_score[2] > avg_score[0]) && (req.giving_up == optimizer_func::optimizer_msgsRequest::NOT_GIVING_UP)) {
    ROS_INFO("The answer is ship the box now");
    res.decision = optimizer_func::optimizer_msgsResponse::PRIORITY_SHIP_THIS;
    res.shipment = priority.shipments[pri_shipment_num];
  } else if (((avg_score[1] > avg_score[2]) && (avg_score[1] > avg_score[0])) || (req.giving_up == optimizer_func::optimizer_msgsRequest::NOT_GIVING_UP)) {
    ROS_INFO("The answer is ship the box now");
    res.decision = optimizer_func::optimizer_msgsResponse::PRIORITY_SHIP_THIS;
    res.shipment = priority.shipments[pri_shipment_num];
  } else {
    res.decision = optimizer_func::optimizer_msgsResponse::USE_CURRENT_BOX;
    res.shipment = priority.shipments[pri_shipment_num];
  }

  return true;
}

#include "optimizer_func/optimizer_func.h"
#include "optimizer_func/helper_funcs.h"

#include "optimizer_func/decider.h"

extern int current_shipment;


// Notes for what to do here.
// Output should be either 0 or 1.  Zero sticks with current box, and one sends the box to Q2.
// The shipment queue can be manipulated here to reflect the desired order of shipment processing.
// There are two conditions that the shipment is being shipped to no one.  In one, a false
// shipment must be created with "ship_to_nowhere" shipment_type, and the shipment queue must
// not lose any shipments.


int deciderQ1(optimizer_func::optimizer_msgs::Request  &req,
			optimizer_func::optimizer_msgs::Response &res) {
  
  ROS_INFO("Beginning optimization for %s", req.loaded.shipment_type.c_str());

  if (shipment_queue.shipments.size() < 2) {
    ROS_INFO("There are not shipments to optimize.");
    return 0;
  }  
  // If this shipment is already a priority shipment, or the only shipment in the queue, there is nothing to optimize.
  if (req.loaded.shipment_type.compare(0, 7, "order_1")) {
    ROS_INFO("This is a shipment for a priority order already, no choice to make.");
    return 0;
  }

  // Which shipments are the priority shipments in the queue
  // Set up an empty shipment 
  osrf_gear::Shipment null_shipment;
  std::vector <osrf_gear::Shipment> pri_ship, pri_loaded, pri_missing, pri_orphaned, pri_reposition;
  for (int indx = 1; indx < shipment_queue.shipments.size(); indx++) {
    if (shipment_queue.shipments[indx].shipment_type.compare(0, 7, "order_1")) {
      pri_loaded.push_back(null_shipment);
      pri_loaded.back().shipment_type = shipment_queue.shipments[indx].shipment_type;
      pri_missing.push_back(null_shipment);
      pri_missing.back().shipment_type = shipment_queue.shipments[indx].shipment_type;
      pri_orphaned.push_back(null_shipment);
      pri_orphaned.back().shipment_type = shipment_queue.shipments[indx].shipment_type;
      pri_reposition.push_back(null_shipment);
      pri_reposition.back().shipment_type = shipment_queue.shipments[indx].shipment_type;
      pri_ship.push_back(shipment_queue.shipments[indx]);
    }
  }
    

  // Count the total parts being ordered. Not doing anything with the infor now.
  int part_counts[the_parts_count];
  memset(&part_counts, 0x00, sizeof(part_counts));
  
  count_parts(current, priority, (int *) &part_counts);

  

  // What is loaded in the form of a shipment message
  osrf_gear::Shipment loaded_parts, cur_loaded, cur_missing, cur_orphaned, cur_reposition;

  // How does the current shipment look
  ROS_DEBUG("Compare current shipment to itself");

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

  // What are the timings for different actions
  double cur_cont_timing = 0.0, cur_comp_timing = 0.0;
  double cur_other_comp_timing = 0.0;

  // Something like this should be done when choosing which shipment to load first.
  // order_breakdown(&(current.shipments[current_shipment]), &loaded_parts, &(current.shipments[current_shipment]), &cur_loaded, &cur_missing, &cur_orphaned, &cur_reposition);
  order_breakdown(&(shipment_queue.shipments[0]), &loaded_parts, &(shipment_queue.shipments[0]), &cur_loaded, &cur_missing, &cur_orphaned, &cur_reposition);
  ROS_DEBUG("----- Continue loading current order current shipment ------");
  cur_cont_timing = timing_breakdown(cur_missing, cur_orphaned, cur_reposition);
  ROS_DEBUG("----- Complete loading current current shipment ------");
  cur_comp_timing = timing_breakdown(shipment_queue.shipments[0], null_shipment, null_shipment);
  
  // How does the current shipment compare to the priority order shipment 0
  // osrf_gear::Shipment pri_loaded, pri_missing, pri_orphaned, pri_reposition;
  // ROS_INFO("Compare current shipment to priority order shipment 0");
  // order_breakdown(&(current.shipments[current_shipment]), &loaded_parts, &(priority.shipments[0]), &pri0_loaded, &pri0_missing, &pri0_orphaned, &pri0_reposition);
  std::vector <double> pri_comp_timing, pri_swap_timing;
  for (int indx = 0; indx < pri_ship.size(); indx++) {
    order_breakdown(&(shipment_queue.shipments[0]), &loaded_parts, &(pri_ship[indx]), &(pri_loaded[indx]), &(pri_missing[indx]), &(pri_orphaned[indx]), &(pri_reposition[indx]));
    pri_swap_timing.push_back(timing_breakdown(pri_missing[indx], pri_orphaned[indx], pri_reposition[indx]));
    pri_comp_timing.push_back(timing_breakdown(pri_ship[indx], null_shipment, null_shipment));
  }

  
  // Which to compare.  Not the best way to choose.
  int pri_shipment_num = 0;
  if ((pri_ship.size() > 1) && (pri_swap_timing[1] < pri_swap_timing[0])) {
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

  // // Continue on current shipment
  // current_order_timings_us[0] = cur_elapsed_time + cur_cont_timing + cur_other_comp_timing + CONVEYOR_TIME;
  // current_order_timings_them[0] = cur_comp_timing + cur_other_comp_timing + CONVEYOR_TIME;
  // ROS_INFO("current_order_timings_us[0]: %4.2f _them[0]: %4.2f", current_order_timings_us[0], current_order_timings_them[0]);

  // // Convert box to another shipment and switch to priority
  // current_order_timings_us[1] = cur_elapsed_time + pri_swap_timing[pri_shipment_num] + cur_comp_timing + cur_other_comp_timing + CONVEYOR_TIME;
  // current_order_timings_them[1] = cur_elapsed_time + pri_swap_timing[pri_shipment_num] + cur_comp_timing + cur_other_comp_timing + CONVEYOR_TIME;

  // // Ship box incomplete and move to priority
  // current_order_timings_us[2] = cur_elapsed_time + pri_comp_timing[0] + pri_comp_timing[1] + cur_comp_timing + cur_other_comp_timing + CONVEYOR_TIME;
  // current_order_timings_them[2] = cur_elapsed_time + pri_comp_timing[0] + pri_comp_timing[1] + cur_comp_timing + cur_other_comp_timing + CONVEYOR_TIME;

  // // Continue on current shipment
  // priority_order_timings[0] = pri_elapsed_time + pri_comp_timing[0] + pri_comp_timing[1] + cur_cont_timing + CONVEYOR_TIME;

  // // Convert box to another shipment and switch to priority
  // priority_order_timings[1] = pri_elapsed_time + pri_swap_timing[pri_shipment_num] + pri_swap_timing[!pri_shipment_num] + CONVEYOR_TIME;

  // // Ship box incomplete and move to priority
  // priority_order_timings[2] = pri_elapsed_time + pri_comp_timing[0] + pri_comp_timing[1] + CONVEYOR_TIME;


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

  return 0;
}


int deciderQ2(optimizer_func::optimizer_msgs::Request  &req,
			optimizer_func::optimizer_msgs::Response &res) {
}

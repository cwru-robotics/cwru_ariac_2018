#include "optimizer_func/optimizer_func.h"

#define CONVEYOR_TIME	45
#define PLCMT		0.875
#define PRIORITY_h	3

// Should auto-populate this soon
char *the_parts[] = {"disk_part", "gasket_part", "gear_part", "piston_rod_part", "pulley_part"};

int count_parts(osrf_gear::Order current, osrf_gear::Order priority, int *parts_counts);
float timing_breakdown(osrf_gear::Shipment to_load, osrf_gear::Shipment to_remove, osrf_gear::Shipment to_move);
int order_breakdown(osrf_gear::Shipment *cur_shipment, osrf_gear::Shipment *loaded, osrf_gear::Shipment *alt_shipment, osrf_gear::Shipment *in_place, osrf_gear::Shipment *to_load, osrf_gear::Shipment *remove, osrf_gear::Shipment *move);

int optimizer_func(osrf_gear::Order current, ros::Time cur_recvd, osrf_gear::Order priority, ros::Time pri_recvd, int current_shipment, osrf_gear::Shipment loaded_parts, ros::Duration delays) {

  // cur_recvd = ros::Time::now() - ros::Duration(load_time(loaded_parts.products.size(), (osrf_gear::Product *) &(loaded_parts)));

  int part_counts[sizeof(the_parts) / sizeof(the_parts[0])];
  memset(&part_counts, 0x00, sizeof(the_parts));

  count_parts(current, priority, (int *) &part_counts);

  ROS_INFO("Part names:    %s %s %s %s %s", the_parts[0], the_parts[1], the_parts[2], the_parts[3], the_parts[4]);

  ROS_INFO("%i\t%i\t%i\t%i\t%i", part_counts[0], part_counts[1], part_counts[2], part_counts[3], part_counts[4]);
  

  // What is loaded in the form of a shipment message
  osrf_gear::Shipment cur_loaded, cur_in_place, cur_to_load, cur_to_remove, cur_to_move;
  // How does the current shipment look
  ROS_INFO("Compare current shipment to itself");
  order_breakdown(&(current.shipments[current_shipment]), &loaded_parts, &(current.shipments[current_shipment]), &cur_in_place, &cur_to_load, &cur_to_remove, &cur_to_move);

  osrf_gear::Shipment pri0_loaded, pri0_in_place, pri0_to_load, pri0_to_remove, pri0_to_move;
  // How does the current shipment compare to the priority order shipment 0
  ROS_INFO("Compare current shipment to priority order shipment 0");
  order_breakdown(&(current.shipments[current_shipment]), &loaded_parts, &(priority.shipments[0]), &pri0_in_place, &pri0_to_load, &pri0_to_remove, &pri0_to_move);

  osrf_gear::Shipment pri1_loaded, pri1_in_place, pri1_to_load, pri1_to_remove, pri1_to_move;
  // How does the corrent shipment compare to the priority order shipment 1
  ROS_INFO("Compare current shipment to priority order shipment 1");
  order_breakdown(&(current.shipments[current_shipment]), &loaded_parts, &(priority.shipments[1]), &pri1_in_place, &pri1_to_load, &pri1_to_remove, &pri1_to_move);

  
  osrf_gear::Shipment null_ship;

  float cur_cont_timing = 0.0, cur_comp_timing = 0.0;
  float cur_other_comp_timing = 0.0;

  ROS_INFO("Part names:    %s %s %s %s %s", the_parts[0], the_parts[1], the_parts[2], the_parts[3], the_parts[4]);

  ROS_INFO("----- Continue loading current order current shipment ------");
  cur_cont_timing = timing_breakdown(cur_to_load, cur_to_remove, cur_to_move);
  ROS_INFO("----- Complete loading current current shipment ------");
  cur_comp_timing = timing_breakdown(current.shipments[current_shipment], null_ship, null_ship);
  if (current.shipments.size() > 1) {
    ROS_INFO("----- Complete loading current order other shipment ------");
    cur_other_comp_timing = timing_breakdown(current.shipments[!current_shipment], null_ship, null_ship);
  }
  
  float pri_comp_timing[] = {0.0, 0.0}, pri_swap_timing[] = {0.0, 0.0};
  ROS_INFO("----- Swap loading priority shipment 0 ------");
  pri_swap_timing[0] = timing_breakdown(pri0_to_load, pri0_to_remove, pri0_to_move);
  ROS_INFO("----- Complete loading priority shipment 0 ------");
  pri_comp_timing[0] = timing_breakdown((priority.shipments[0]), null_ship, null_ship);

  if (priority.shipments.size() > 1) {
    ROS_INFO("----- Swap loading priority shipment 1 ------");
    pri_swap_timing[1] = timing_breakdown(pri1_to_load, pri1_to_remove, pri1_to_move);
    ROS_INFO("----- Complete loading priority shipment 1 ------");
    pri_comp_timing[1] = timing_breakdown((priority.shipments[1]), null_ship, null_ship);
  }
  
  // Which to compare
  int pri_shipment_num = 0;
  if ((priority.shipments.size() > 1) && (pri_swap_timing[1] < pri_swap_timing[0])) {
    pri_shipment_num = 1;
  }
  ROS_INFO(" ****** Comparing current order shipment to priority order shipment %i ******", pri_shipment_num);
  
  // Calculate the TS (Total Scores)

  float cf = 1700.0 / 2100.0;
  float current_order_timings_us[] = {0.0, 0.0, 0.0};
  float current_order_timings_them[] = {0.0, 0.0, 0.0};
  float priority_order_timings[] = {0.0, 0.0, 0.0};

  float cur_elapsed_time = (ros::Time::now().toSec() - cur_recvd.toSec());
  float pri_elapsed_time = (ros::Time::now().toSec() - pri_recvd.toSec());
  
  current_order_timings_us[0] = cur_elapsed_time + cur_cont_timing + delays.toSec() + cur_other_comp_timing + CONVEYOR_TIME;
  current_order_timings_them[0] = cur_comp_timing + cur_other_comp_timing + CONVEYOR_TIME;

  current_order_timings_us[1] = cur_elapsed_time + pri_swap_timing[pri_shipment_num] + cur_comp_timing + cur_other_comp_timing + CONVEYOR_TIME;
  current_order_timings_them[1] = cur_elapsed_time + pri_swap_timing[pri_shipment_num] + cur_comp_timing + cur_other_comp_timing + CONVEYOR_TIME;

  current_order_timings_us[2] = cur_elapsed_time + pri_comp_timing[0] + pri_comp_timing[1] + cur_comp_timing + cur_other_comp_timing + CONVEYOR_TIME;
  current_order_timings_them[2] = cur_elapsed_time + pri_comp_timing[0] + pri_comp_timing[1] + cur_comp_timing + cur_other_comp_timing + CONVEYOR_TIME;

  priority_order_timings[0] = pri_elapsed_time + pri_comp_timing[0] + pri_comp_timing[1] + cur_cont_timing + CONVEYOR_TIME;

  priority_order_timings[1] = pri_elapsed_time + pri_swap_timing[pri_shipment_num] + pri_swap_timing[!pri_shipment_num] + CONVEYOR_TIME;

  priority_order_timings[2] = pri_elapsed_time + pri_comp_timing[0] + pri_comp_timing[1] + CONVEYOR_TIME;


  float total_score[4][4];
  memset(&total_score, 0x00, sizeof(total_score));
  float completion_score[3];
  float f_delays = delays.toSec();

  completion_score[0] = (2 + PLCMT) * (float) current.shipments[0].products.size() + ((current.shipments.size() > 1) ? (float) current.shipments[1].products.size() : 0.0);
  completion_score[1] = PRIORITY_h * (2 + PLCMT) * (float) priority.shipments[0].products.size() + ((priority.shipments.size() > 1) ? (float) priority.shipments[1].products.size() : 0.0);
  completion_score[2] = (completion_score[0] + completion_score[1]) / 2.0;

  for (int row = 0; row <= sizeof(priority_order_timings)/sizeof(priority_order_timings[0]); row++) {
    for (int col = 0; col <= sizeof(priority_order_timings)/sizeof(priority_order_timings[0]); col++) {
      float tempa = cf * completion_score[2];
      float tempb = (current_order_timings_us[col] + f_delays) / (current_order_timings_them[row]) * completion_score[0];
      float tempc = (priority_order_timings[col] + f_delays) / (priority_order_timings[row]) * completion_score[1];

      total_score[row][col] =  tempa +  tempb + tempc;
    }
  }
  ROS_INFO("%4.2f\t%4.2f\t%4.2f", total_score[0][0], total_score[0][1], total_score[0][2]);
  ROS_INFO("%4.2f\t%4.2f\t%4.2f", total_score[1][0], total_score[1][1], total_score[1][2]);
  ROS_INFO("%4.2f\t%4.2f\t%4.2f", total_score[2][0], total_score[2][1], total_score[2][2]);


  float avg_score[] = {(total_score[1][0] + total_score[2][0]) / 2.0, (total_score[0][1] + total_score[2][1]) / 2.0, (total_score[0][2] + total_score[1][2])/ 2.0};
  
  ROS_INFO("Decision Time...");
  ROS_INFO("%2.4f\t%2.4f\t%2.4f", avg_score[0], avg_score[1], avg_score[2]);

  if ((avg_score[2] > avg_score[1]) && (avg_score[2] > avg_score[0]))
    return ABANDON_CURRENT_ORDER;
  else if ((avg_score[1] > avg_score[2]) && (avg_score[1] > avg_score[0]))
    return SWAP_TO_PRIORITY_ORDER;
  else
    return FILL_CURRENT_ORDER;
}

int count_parts(osrf_gear::Order current, osrf_gear::Order priority, int *parts_counts) {
  int part = 0;
  
  for (int indx = 0; indx < current.shipments.size(); indx++) {
    for (int indy = 0; indy < current.shipments[indx].products.size(); indy++) {
      for (part = 0; part < sizeof(the_parts) / sizeof(the_parts[0]); part++) {
	if (!current.shipments[indx].products[indy].type.compare(the_parts[part]))
	  break;
      }
      *(parts_counts+part) += 1;
    }
  }

  for (int indx = 0; indx < priority.shipments.size(); indx++) {
    for (int indy = 0; indy < priority.shipments[indx].products.size(); indy++) {
      for (part = 0; part < sizeof(the_parts) / sizeof(the_parts[0]); part++) {
	if (!priority.shipments[indx].products[indy].type.compare(the_parts[part]))
	  break;
      }
      *(parts_counts+part) += 1;
    }
  }

  return 0;
}

float timing_breakdown(osrf_gear::Shipment to_load, osrf_gear::Shipment to_remove, osrf_gear::Shipment to_move) {

  float unload_time = 0.0;
  float new_load_time = 0.0;
  float swapping_time = 0.0;
  float total_time = 0.0;

  unload_time += dump_time(to_remove.products.size(), (osrf_gear::Product *) &(to_remove.products));

  // How much time to swap parts
  swapping_time = swap_time((int) to_move.products.size(), (osrf_gear::Product *) &(to_move.products));

  // How much time to load any remaining parts
  new_load_time = 0;

  new_load_time += load_time(to_load.products.size(), (osrf_gear::Product *) &(to_load.products));

  // Total time to swap over order.
  total_time = unload_time + swapping_time + new_load_time;
  
  ROS_INFO("Dump parts time: %1.4f", unload_time);
  ROS_INFO("Swap out parts time: %1.4f", swapping_time);
  ROS_INFO("Additional parts loading time: %1.4f", new_load_time);
  ROS_INFO("Total time: %1.4f", total_time);

  return total_time;
}
			     
int order_breakdown(osrf_gear::Shipment *cur_shipment, osrf_gear::Shipment *loaded, osrf_gear::Shipment *alt_shipment, osrf_gear::Shipment *in_place, osrf_gear::Shipment *to_load, osrf_gear::Shipment *remove, osrf_gear::Shipment *move) {

    
  int part = 0, loaded_wrong = 0;

  int num_parts_ordered[sizeof(the_parts)/sizeof(the_parts[0])]; memset(&num_parts_ordered, 0x00, sizeof(num_parts_ordered));
  int num_parts_loaded[sizeof(the_parts)/sizeof(the_parts[0])]; memset(&num_parts_loaded, 0x00, sizeof(num_parts_loaded));
  int num_each_part[sizeof(the_parts)/sizeof(the_parts[0])]; memset(&num_each_part, 0x00, sizeof(num_each_part));
  int num_each_part_conversion[sizeof(the_parts)/sizeof(the_parts[0])]; memset(&num_each_part_conversion, 0x00, sizeof(num_each_part_conversion));

  *to_load = *cur_shipment;
  
  for (int part = 0; part < sizeof(the_parts)/sizeof(char*); part ++) {

    // How many of each part are in first order shipment
    for(int prod = 0; prod < cur_shipment->products.size(); prod++) {
      num_parts_ordered[part] += cur_shipment->products[prod].type.compare(the_parts[part]) ? 0 : 1;
    }

    // How many of each part are in alternate order shipment
    for(int prod = 0; prod < alt_shipment->products.size(); prod++) {
      num_each_part[part] += (alt_shipment->products[prod].type.compare(the_parts[part])) ? 0 : 1;
    }

    // How many of each part has been loaded
    for (int prod = 0; prod < loaded->products.size(); prod++) {
      num_parts_loaded[part] += loaded->products[prod].type.compare(the_parts[part]) ? 0 : 1;
    }

    // How many parts must added or removed by part
    num_each_part_conversion[part] = num_each_part[part] - num_parts_loaded[part];

  }

  ROS_INFO("Parts ordered:     %i %i %i %i %i", num_parts_ordered[0], num_parts_ordered[1], num_parts_ordered[2], num_parts_ordered[3], num_parts_ordered[4]);
  ROS_INFO("Alt parts ordered: %i %i %i %i %i", num_each_part[0], num_each_part[1], num_each_part[2], num_each_part[3], num_each_part[4]);
  ROS_INFO("Parts loaded:      %i %i %i %i %i", num_parts_loaded[0], num_parts_loaded[1], num_parts_loaded[2], num_parts_loaded[3], num_parts_loaded[4]);
  ROS_INFO("Conversion:        %i %i %i %i %i", num_each_part_conversion[0], num_each_part_conversion[1], num_each_part_conversion[2], num_each_part_conversion[3], num_each_part_conversion[4]);

  // Iterate over all the loaded parts
  for (int indx = 0; indx < loaded->products.size(); indx++) {

    // What is the current part
    for (part = 0; part < sizeof(the_parts)/sizeof(the_parts[0]); part++) {
      if (!loaded->products[indx].type.compare(the_parts[part]))
	break;
    }
    
    // Iterate over all the parts in one of the alternate shipments
    for (int indy = alt_shipment->products.size()-1; indy > 0; --indy) {
      // Only do this for the specific parts we are checking.
      if (!loaded->products[indx].type.compare(the_parts[part])){
	loaded_wrong = 0;
	// If the current loaded part being examined is the correct type
	if (!loaded->products[indx].type.compare(alt_shipment->products[indy].type)) {
	  loaded_wrong = 1;
	  if ( (sqrt(pow(cur_shipment->products[indy].pose.position.x - loaded->products[indx].pose.position.x, 2) + \
		     pow(cur_shipment->products[indy].pose.position.y - loaded->products[indx].pose.position.y, 2)) < T_OFFSET) && \
	       (fabs(cur_shipment->products[indy].pose.orientation.z - loaded->products[indx].pose.orientation.z) < O_OFFSET)) {
	    // It is loaded correctly, keep track of it and no need to do any more comparing
	    in_place->products.push_back(loaded->products[indx]);
	    // Pop off this part from the order.
	    to_load->products.pop_back();
	    break;
	  }
	} 
	
	if (num_each_part_conversion[part] < 0) {
	  num_each_part_conversion[part]++;
	  // This is an overload.  Add it to a list to remove.
	  remove->products.push_back(loaded->products[indx]);
	}
	if ((num_each_part_conversion[part] == 0) && loaded_wrong) {
	  // Add this to a list to move.
	  move->products.push_back(loaded->products[indx]);
	  // Pop off this part from the order.
	  to_load->products.pop_back();
	}
      } 
    }
  }

  ROS_INFO("Inplace, load, remove, move");
  ROS_INFO("%i %i %i %i", (int) (*in_place).products.size(), (int) (*to_load).products.size(), (int) (*remove).products.size(), (int) (*move).products.size());

  return 0;

}

float load_time(int num_prods, osrf_gear::Product *prod) {
  float amt_time = 0.0;

  for (int indx = 0; indx < num_prods; indx++) {
    amt_time += 45.0;
  }
  
  return amt_time;
}


float dump_time(int num_prods, osrf_gear::Product *prod) {
  float amt_time = 0.0;

  for (int indx = 0; indx < num_prods; indx++) {
    amt_time += 15.0;
  }
  
  return amt_time;
}

float swap_time(int num_prods, osrf_gear::Product *prods) {
  float amt_time = 0.0;

  // Towers of Hanoi solution.  Likely way too harsh, but good for now.
  amt_time = (num_prods > 0.0) ? ((2 * num_prods) - 1) * 40.0 : 0.0;
  
  return amt_time;
}

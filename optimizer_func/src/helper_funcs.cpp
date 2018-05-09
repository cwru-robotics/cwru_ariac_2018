#include "optimizer_func/helper_funcs.h"

// Should auto-populate this soon
char *the_parts[] = {"disk_part", "gasket_part", "gear_part", "piston_rod_part", "pulley_part"};
int the_parts_count = 5;

osrf_gear::Order shipment_queue;
int shipment_queue_indx = 0;


int count_parts(osrf_gear::Order current, osrf_gear::Order priority, int *parts_counts) {
  int part = 0;
  
  for (int indx = 0; indx < current.shipments.size(); indx++) {
    for (int indy = 0; indy < current.shipments[indx].products.size(); indy++) {
      for (part = 0; part < the_parts_count; part++) {
	if (!current.shipments[indx].products[indy].type.compare(the_parts[part]))
	  break;
      }
      *(parts_counts+part) += 1;
    }
  }

  for (int indx = 0; indx < priority.shipments.size(); indx++) {
    for (int indy = 0; indy < priority.shipments[indx].products.size(); indy++) {
      for (part = 0; part < the_parts_count; part++) {
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
  
  ROS_DEBUG("Dump parts time: %1.4f", unload_time);
  ROS_DEBUG("Swap out parts time: %1.4f", swapping_time);
  ROS_DEBUG("Additional parts loading time: %1.4f", new_load_time);
  ROS_DEBUG("Total time: %1.4f", total_time);

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

  ROS_DEBUG("Parts ordered:     %i %i %i %i %i", num_parts_ordered[0], num_parts_ordered[1], num_parts_ordered[2], num_parts_ordered[3], num_parts_ordered[4]);
  ROS_DEBUG("Alt parts ordered: %i %i %i %i %i", num_each_part[0], num_each_part[1], num_each_part[2], num_each_part[3], num_each_part[4]);
  ROS_DEBUG("Parts loaded:      %i %i %i %i %i", num_parts_loaded[0], num_parts_loaded[1], num_parts_loaded[2], num_parts_loaded[3], num_parts_loaded[4]);
  ROS_DEBUG("Conversion:        %i %i %i %i %i", num_each_part_conversion[0], num_each_part_conversion[1], num_each_part_conversion[2], num_each_part_conversion[3], num_each_part_conversion[4]);

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

  ROS_DEBUG("Inplace, load, remove, move");
  ROS_DEBUG("%i %i %i %i", (int) (*in_place).products.size(), (int) (*to_load).products.size(), (int) (*remove).products.size(), (int) (*move).products.size());

  return 0;

}

float load_time(int num_prods, osrf_gear::Product *prod) {
  float amt_time = 0.0;

  for (int indx = 0; indx < num_prods; indx++) {
    amt_time += LOAD_TIME;
  }
  
  return amt_time;
}

float dump_time(int num_prods, osrf_gear::Product *prod) {
  float amt_time = 0.0;

  for (int indx = 0; indx < num_prods; indx++) {
    amt_time += DUMP_TIME;
  }
  
  return amt_time;
}

float swap_time(int num_prods, osrf_gear::Product *prods) {
  float amt_time = 0.0;

  // Towers of Hanoi solution.  Likely way too harsh, but good for now.
  amt_time = (num_prods > 0.0) ? ((2 * num_prods) - 1) * MOVE_TIME : 0.0;
  
  return amt_time;
}


#ifndef HELPER_FUNCS_H
#define HELPER_FUNCS_H

#include "ros/ros.h"
#include "ros/time.h"
#include <time.h>

#include "osrf_gear/Order.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/Product.h"

#include "math.h"

#include "optimizer_func/optimizer_msgs.h"
#include "optimizer_func/optimizer_msgsRequest.h"
#include "optimizer_func/optimizer_msgsResponse.h"

#define T_OFFSET	0.03 // 3 cm allowable part translation offset
#define O_OFFSET	0.1  // 0.1 radian allowable part orientation offset

#define LOAD_TIME	45.0 // Amount of time it takes to retrieve a part from a bin
#define DUMP_TIME	15.0 // Amount of time it takes to get a part out of the box
#define MOVE_TIME	15.0 // Amount of time it takes to move a part around in the box

#define CONVEYOR_TIME	45.0 // Amount of time it take the box to move from Q1 to the quadrotor
#define PLCMT		0.875 // The percentage of correct placements
#define PRIORITY_h	3 // The multiplier for priority orders

// Should auto-populate this soon
extern char *the_parts[];
extern int the_parts_count;

int count_parts(osrf_gear::Order current, osrf_gear::Order priority, int *parts_counts);
float timing_breakdown(osrf_gear::Shipment to_load, osrf_gear::Shipment to_remove, osrf_gear::Shipment to_move);
int order_breakdown(osrf_gear::Shipment *cur_shipment, osrf_gear::Shipment *loaded, osrf_gear::Shipment *alt_shipment, osrf_gear::Shipment *in_place, osrf_gear::Shipment *to_load, osrf_gear::Shipment *remove, osrf_gear::Shipment *move);


float load_time(int num_prods, osrf_gear::Product *prod);
float dump_time(int num_prods, osrf_gear::Product *prod);
float swap_time(int num_prods, osrf_gear::Product *prods);

#endif

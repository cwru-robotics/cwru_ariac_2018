
#ifndef OPTIMIZER_FUNC_H
#define OPTIMIZER_FUNC_H

#include "ros/ros.h"

#include "ros/time.h"
#include <time.h>

#include "osrf_gear/Order.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/Product.h"

#include "math.h"

#define	FILL_CURRENT_ORDER 	0
#define SWAP_TO_PRIORITY_ORDER 	1
#define ABANDON_CURRENT_ORDER 	2

#define T_OFFSET	0.03 // 3 cm allowable part translation offset
#define O_OFFSET	0.1  // 0.1 radian allowable part orientation offset

int optimizer_func(osrf_gear::Order current, ros::Time cur_recvd, osrf_gear::Order priority, ros::Time pri_recv_d, int current_shipment, osrf_gear::Shipment loaded_parts, ros::Duration delays);

float load_time(int num_prods, osrf_gear::Product *prod);
float dump_time(int num_prods, osrf_gear::Product *prod);
float swap_time(int num_prods, osrf_gear::Product *prods);


#endif // OPTIMIZER_FUNC_H


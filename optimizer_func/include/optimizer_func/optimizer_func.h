
#ifndef OPTIMIZER_FUNC_H
#define OPTIMIZER_FUNC_H

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

extern osrf_gear::Order current, priority;
extern ros::Time current_order_recvd, priority_order_recvd;

extern osrf_gear::Order shipment_queue;
extern int shipment_queue_indx;

extern std::vector <osrf_gear::Shipment> shipping_now;

extern int alert_level;

#define LEVEL_GREEN	0
#define LEVEL_YELLOW	1
#define LEVEL_RED	2
#define LEVEL_COMPLETE	3

#define NULL_SHIPMENT "empty_shipment"

bool optimize_shipments(optimizer_func::optimizer_msgs::Request  &req,
			optimizer_func::optimizer_msgs::Response &res);
//int decision_func(osrf_gear::Shipment loaded_parts, ros::Duration delays);

#endif // OPTIMIZER_FUNC_H


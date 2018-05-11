#include "optimizer_func/optimizer_func.h"
#include "optimizer_func/helper_funcs.h"

#include "optimizer_func/decider.h"


int deciderQ1(optimizer_func::optimizer_msgs::Request  &req,
			optimizer_func::optimizer_msgs::Response &res) {
  ROS_INFO("Dumb optimizer for Q1.");
  return 0;
}

int deciderQ2(optimizer_func::optimizer_msgs::Request  &req,
			optimizer_func::optimizer_msgs::Response &res) {
  ROS_INFO("Dumb optimizer for Q2.");
  return 0;
}

//fnc to illustrate how to use "map" to convert strings to ID numbers
//if string is unrecognized, index returns 0

#include <ros/ros.h>
//mappings are defined in this header file:
//map "mappings" will be global
#include "part_name_to_id.h"



int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "part_name_to_id_test_main"); //node name

    char name1[]="gear_part";
    char name2[]="piston_rod_part";
    char name3[]="pulley_part";

    int part_id;

    part_id = mappings[name1];

    ROS_INFO("ID for %s is %d",name1,part_id);
    ROS_INFO("ID for %s is %d",name2,mappings[name2]);
    ROS_INFO("ID for %s is %d",name3,mappings[name3]);
return 0;
} 

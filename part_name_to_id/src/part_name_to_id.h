//header file "map" to convert strings to ID numbers
//if string is unrecognized, index returns 0
//see example usage in part_name_to_id.cpp
#ifndef PART_NAME_TO_ID_H_
#define PART_NAME_TO_ID_H_
#include <map>
#include <string>


std::map<std::string, int> mappings =
{
   {"gear_part",1},
   {"piston_rod_part",2}
};

#endif


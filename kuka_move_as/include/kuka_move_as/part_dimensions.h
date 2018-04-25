//part frame notes:
// GASKET: frame is centered, but with origin at BOTTOM surface; x-axis points towards pin feature
// PISTON ROD: frame origin is at BOTTOM surface of part, centered in hole; y-axis points towards opposite end of rod
// GEAR: origin is at BOTTOM of part (centered), and Y-AXIS points towards pin feature
//DISK: origin is at BOTTOM of part (centered), pin feature is 45 deg (between x-axis and y_axis)
#ifndef PART_DIMENSIONS_H
#define PART_DIMENSIONS_H
const double PISTON_ROD_PART_THICKNESS= 0.007; // wsn modified for qual3; 0.0075; //works for qual2
const double GEAR_PART_THICKNESS = 0.0124; // modified for qual3; 0.015; // 0.015 works for qual2
const double DISK_PART_THICKNESS = 0.0247;  //note sure about this one...maybe OK
const double GASKET_PART_THICKNESS = 0.02; // 0.0336; //wsn change for gear 1.1
//per  /opt/ros/indigo/share/osrf_gear/models/pulley_part_ariac, pulley-part collision model should be 0.0720 thick
const double PULLEY_PART_THICKNESS = 0.0728;  //0.7255 = z on bin; 0.7983 on top of another pulley: 0.0728 thickness; origin on bottom

//here are some hand-tuned "kludge" parameters to tweak grasp transforms;
const double GEAR_PART_GRASP_Y_OFFSET = 0.0; //0.04;  TRY ZERO OFFSET
const double DISK_PART_GRASP_Z_OFFSET = 0.005;
const double GASKET_PART_GRASP_Z_OFFSET = 0.0; //0.006;
const double GASKET_PART_GRASP_X_OFFSET = 0.03;
const double PULLEY_PART_GRASP_Z_OFFSET = 0.007; //0.01; //0.005;

//surface heights:
const double BASE_LINK_X_COORD = -0.05; //-0.05
const double BASE_LINK_HEIGHT =  0.700; //0.700


const double BOX_HEIGHT = 0.460; //0.4446;  //try elevating box height to provide dropoff clearance
#endif
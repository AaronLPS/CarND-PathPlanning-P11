#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <string>
#include "basic_types.h"

//define the path of the map file: highway map waypoints
//highway_map.csv: The waypoints are in the middle of the double-yellow dividing line in the center of the highway
extern std::string map_file_;

const double MAP_S_MAX =  6945.554; // The max s value of the map waypoins
const double DETECT_S_FOV = 70.0;  //Field Of View on s of Frenet coordinates

const double LANE_WIDTH = 4.0; //4m

const double DT_PER_POINT = 0.02; //  time interval: 0.02 s per waypoint (map)

const double CAR_MAX_SPEED_MPH = 49; //MPH

const double CAR_MAX_ACCELERATION = 9.0; // m/s^2
const double CAR_MAX_SPEED = 22.0; // m/s
//extra margin for safety: safety box
const double SAFETY_MARGIN_LATERAL = 3.0; // m
const double SAFETY_MARGIN_LONGITUDINAL = 6.0; // m

const int HIGHWAY_LANE_NUMBERS = 3; //3 lanes

// default safety distance for lane change
const double SAFETY_DISTANCE_LANE_CHANGE = 10.0;


//TODO


//END TODO




#endif // CONFIGURATION_H

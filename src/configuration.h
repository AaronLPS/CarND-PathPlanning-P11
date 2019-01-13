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

const double CAR_MAX_SPEED_MPH = 47; //MPH ~ 21 m/s

const double CAR_MAX_ACCELERATION = 10.0; // m/s^2
const double CAR_MAX_SPEED = 21.0; // m/s
const double CAR_MAX_JERK  = 10; // m/s^3
//extra margin for safety: safety box
const double SAFETY_MARGIN_LATERAL = 3.0; // m
const double SAFETY_MARGIN_LONGITUDINAL = 6.0; // m
const int MAX_COLLISION_STEP = 25; // detect collision in this range(waypoints) in front


const int HIGHWAY_LANE_NUMBERS = 3; //3 lanes

// default safety distance for lane change
const double SAFETY_DISTANCE_LANE_CHANGE = 10.0;

// max speed increase  per  time interval
const double MAX_SPEED_INC = CAR_MAX_ACCELERATION * DT_PER_POINT; // m/s per 0.02 sec
const double MAX_SPEED_INC_MPH = ms_to_mph(MAX_SPEED_INC);

// Trajectory Layer
const bool TRAJECTORY_JMT_ENABLE = true;
const int TRAJECTORY_WAYPOINTS_NUMBER = 50;

// assume 100 ms max simulator latency
const int REACTION_LATENCY_WAYPOINTS = 5; // waypoints

//weighted cost function: high weight with high critical
const int WEIGHT_COST_FEASIBILITY = 10000; // vs collisions, vs vehicle capabilities
const int WEIGHT_COST_SAFETY      = 1000; // vs buffer distance, vs visibility or curvature
const int WEIGHT_COST_LEGALITY    = 100; // vs speed limits
const int WEIGHT_COST_COMFORT     = 10; // vs jerk
const int WEIGHT_COST_EFFICIENCY  = 1; // vs target lane, target speed and time to goal

const double MAX_S = 6945.554;

// center point of the track
const double TRACK_CENTER_X = 1000;
const double TRACK_CENTER_Y = 2000;


#endif // CONFIGURATION_H

#ifndef BASIC_TYPES_H
#define BASIC_TYPES_H

#include <vector>

using namespace std;

/* @note sensor_fusion: The data format for each car is: [ id, x, y, vx, vy, s, d]. The id is a unique identifier for that car.
* The x, y values are in global map coordinates, and the vx, vy values are the velocity components, also in reference to the global map.
* s and d are the Frenet coordinates for that car.
*
* vector<vector<double>> const &sensor_fusion
* */
typedef vector<vector<double>> SensorFusionType;

/* @note CarStates contains all infomations related to the car
 * The x, y values are in global map coordinates
 * s and d are the Frenet coordinates
 *
 * */
struct CarStates {
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
    double speed_target;
    int    lane;
    bool   emergency;
    CarStates (double X=0, double Y=0, double S=0, double D=0, double YAW=0,
               double V=0, double VF=0, double L=0, bool E=false): x(X), y(Y), s(S), d(D), yaw(YAW), speed(V),
      speed_target(VF), lane(L), emergency(E) {}
};


struct EuclideanCoord{
    double x;
    double y;
};

struct FrenetCoord{
  double s;
  double d;
};

double mph_to_ms(double mph);
double ms_to_mph(double ms);
double get_dcenter(int lane);
double deg2rad(double x);
double rad2deg(double x);

#endif // BASIC_TYPES_H

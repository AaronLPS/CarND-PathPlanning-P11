#include "basic_types.h"
#include "configuration.h"

#define M_PI        3.14159265358979323846264338327950288   /* pi             */

double mph_to_ms(double mph) {
    return mph / 2.24;   // m/s
}


double ms_to_mph(double ms) {
    return ms * 2.24;  // mph
}

// d coord for center lane
double get_dcenter(int lane) {
  double dcenter = (lane + 0.5) * LANE_WIDTH;
  if (dcenter >= 10) {
    // this a workaround for a simulator issue I think (reported by others as well on udacity forums)
    // with d set to 10 from time to time a lane violation is reported by simulator
    // while everything looks fine
    dcenter = 9.8;
  }
  return dcenter;
}


double deg2rad(double x) {
    return x * M_PI / 180;
}

double rad2deg(double x) {
    return x * 180 / M_PI;
}

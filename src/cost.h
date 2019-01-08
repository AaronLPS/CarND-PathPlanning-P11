#ifndef COST_H
#define COST_H

#include <math.h>
#include <iostream>
#include <vector>
#include <map>
#include <cassert>
#include <cmath>

#include "utility.h"
#include "params.h"
#include "prediction.h"
#include "Eigen-3.3/Eigen/Dense"

#include "behaviour.h"
#include "trajectory.h"



class Cost {
public:
  Cost(struct TrajectoryXY const &trajectory, BehaviourTarget target, Prediction &predictions, int car_lane);
  virtual ~Cost();

  double get_cost();

private:
  bool check_collision(double x0, double y0, double theta0, double x1, double y1, double theta1);
  int  check_collision_on_trajectory(struct TrajectoryXY const &trajectory, std::map<int, vector<EuclideanCoord> > &predictions);
  bool check_max_capabilities(std::vector<std::vector<double>> &traj);
  double get_predicted_dmin(struct TrajectoryXY const &trajectory, std::map<int, std::vector<EuclideanCoord> > &predictions);

  double cost_;
};

#endif // COST_H

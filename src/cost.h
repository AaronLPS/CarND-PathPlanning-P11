#ifndef COST_H
#define COST_H

#include <math.h>
#include <iostream>
#include <vector>
#include <map>
#include <cassert>
#include <cmath>

#include "basic_types.h"
#include "configuration.h"

#include "prediction.h"
#include "Eigen-3.3/Eigen/Dense"

#include "behaviour.h"
#include "trajectory.h"



class Cost {
public:
  Cost(struct TrajectoryXY const &trajectory, BehaviourTarget target, Prediction &prediction, int car_lane);
  virtual ~Cost();

  double get_cost();

private:
  bool CollisionDetectionSAT(double x0, double y0, double theta0, double x1, double y1, double theta1);
  int  DetectCollision(struct TrajectoryXY const &trajectory, map<int, vector<EuclideanCoord> > &predictions);
  bool check_max_capabilities(vector<vector<double>> &traj);
  double get_predicted_dmin(struct TrajectoryXY const &trajectory, map<int, vector<EuclideanCoord> > &predictions);

private:
  double trajectory_cost;
};

#endif // COST_H

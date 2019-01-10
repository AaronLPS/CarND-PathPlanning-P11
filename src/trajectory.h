#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>
#include <iostream>
#include <vector>

#include "map.h"
#include "behaviour.h"
#include "spline.h"
#include "basic_types.h"
#include "map.h"
#include "cost.h"
#include "configuration.h"
#include "prediction.h"
#include "Eigen-3.3/Eigen/Dense"

using namespace std;

struct TrajectoryXY {
    vector<double> path_x;
    vector<double> path_y;
    TrajectoryXY (vector<double> X={}, vector<double> Y={}) : path_x(X), path_y(Y) {}
};

// Point of a C2 class function
struct PointC2 {
  double f;
  double f_dot;
  double f_ddot;
  PointC2 (double y=0, double y_dot=0, double y_ddot=0) : f(y), f_dot(y_dot), f_ddot(y_ddot) {}
};

struct TrajectorySD {
  vector<PointC2> path_s;
  vector<PointC2> path_d;
  TrajectorySD (vector<PointC2> S={}, vector<PointC2> D={}) : path_s(S), path_d(D) {}
};

struct TrajectoryJMT {
  TrajectoryXY trajectory;
  TrajectorySD path_sd;
};


struct PreviousPath {
  TrajectoryXY xy;   // < waypoints number (some already used by simulator)
  TrajectorySD sd;   // exactly PARAM_NB_POINTS (not sent to simulator)
  int num_xy_reused;  // reused from xy
  PreviousPath (TrajectoryXY XY={}, TrajectorySD SD={}, int N=0) : xy(XY), sd(SD), num_xy_reused(N) {}
};


class Trajectory {
public:
  Trajectory(vector<BehaviourTarget> targets, Map &map, CarStates &car, PreviousPath &previous_path, Prediction &predictions);
  ~Trajectory() {}

  double getMinCost() { return min_cost; }
  double getMinCostIndex() { return min_cost_index; }
  TrajectoryXY getMinCostTrajectoryXY() { return trajectories_list[min_cost_index]; }
  TrajectorySD getMinCostTrajectorySD() { return trajectories_sd_list[min_cost_index]; }


private:
  TrajectoryJMT GenerateTrajectoryEmergency(BehaviourTarget target, Map &map, CarStates const &car, PreviousPath const &previous_path);
  TrajectoryJMT GenerateTrajectoryJMT(BehaviourTarget target, Map &map, PreviousPath const &previous_path);

  vector<double> JMT(vector< double> start, vector <double> end, double T);
  double calculate_polynomial(vector<double> c, double t);
  double calculate_polynomial_dot(vector<double> c, double t);
  double calculate_polynomial_ddot(vector<double> c, double t);

  TrajectoryXY GenerateTrajectorySPLINE (BehaviourTarget target, Map &map, CarStates const &car, PreviousPath const &previous_path);


private:
  vector<class Cost> costs_;
  vector<TrajectoryXY> trajectories_list;
  vector<TrajectorySD> trajectories_sd_list;
  double min_cost;
  int min_cost_index;
};



TrajectoryJMT JMT_init(double car_s, double car_d);


#endif // TRAJECTORY_H

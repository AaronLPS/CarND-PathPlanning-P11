#ifndef BEHAVIOUR_H
#define BEHAVIOUR_H

#include <math.h>
#include <iostream>
#include <vector>

#include "basic_types.h"
#include "configuration.h"
#include "prediction.h"

using namespace std;

struct BehaviourTarget {
  double lane;
  double velocity;  // for JMT(Jerk Minimum Trajectory)
  double time;      // for manoeuvre
  double acceleration; // for emergency trajectories
  BehaviourTarget(double L=0, double V=0, double T=0, double A=0) : lane(L), velocity(V), time(T), acceleration(A) {}
};

class Behaviour
{
public:
    Behaviour(SensorFusionType const &sensor_fusion, const CarStates &car_states, Prediction const &prediction);
    virtual ~Behaviour();
    vector<BehaviourTarget> OutputBehaviourTarget();
private:
  double get_d_left(int lane);
  double get_d_right(int lane);

  private:
    vector<BehaviourTarget> behaviour_target_list;
};

#endif // BEHAVIOUR_H

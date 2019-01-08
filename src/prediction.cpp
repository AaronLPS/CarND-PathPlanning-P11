#include "prediction.h"

/* @brief predict the position of surrounding(front and back) cars
 *
 * @param sensor_fusion: input detected objects' states: [ id, x, y, vx, vy, s, d]
 * @note sensor_fusion: The data format for each car is: [ id, x, y, vx, vy, s, d]. The id is a unique identifier for that car.
 * The x, y values are in global map coordinates, and the vx, vy values are the velocity components, also in reference to the global map.
 * s and d are the Frenet coordinates for that car.
 *
 * @param car_states: the autonomous vehicel (ego)
 * @param vision_range: how many waypoints will be involved in the prediction range
 *
 * */
Prediction::Prediction(SensorFusionType const &sensor_fusion, CarStates const &car_states, int vision_range){

    //get detected objects' index from sensor fusion
    vector<int> detected_object_index;
    detected_object_index = DetectObjects(sensor_fusion, car_states);

    for(int i=0; i< detected_object_index.size(); i++){
        int object_index = detected_object_index[i];
        if(object_index >= 0){//if detected there is an object
            //start predict the position of each object with time and vision_range
            vector<EuclideanCoord> xy_predicted;
            for(int k=0; k<vision_range; k++){
                EuclideanCoord xy_coord;
                //x' = x + v*t
                xy_coord.x = sensor_fusion[object_index][1] + sensor_fusion[object_index][3] * k * DT_PER_POINT;
                //y' = y + v*t
                xy_coord.y = sensor_fusion[object_index][2] + sensor_fusion[object_index][4] * k * DT_PER_POINT;
                xy_predicted.push_back(xy_coord);
            }
            xy_predicted_in_vision[object_index] = xy_predicted;
        }
    }

    SetSafetyDistances(sensor_fusion, car_states);
    SetLaneInfo(sensor_fusion, car_states);
}



Prediction::~Prediction(){

}


/* @brief detect surrounding objects using sensor fusion outputs and states of ego car
 * @note only detect the closest objects, max 6 = 3 front + 3 back, one for each lane
 *
 * @param sensor_fusion
 * @param car_states
 * @return vector<int>: index of the detected objects
 * */
vector<int> Prediction::DetectObjects(SensorFusionType const &sensor_fusion, CarStates const &car_states) {

    double s_fov_min = car_states.s - DETECT_S_FOV;
    double s_fov_max = car_states.s + DETECT_S_FOV;
    //deal with s wraparound
    double s_fov_shift = 0;
    if(s_fov_min < 0){
        s_fov_shift = -s_fov_min;
    }
    if(s_fov_max > MAP_S_MAX){
        s_fov_shift = MAP_S_MAX - s_fov_max;
    }
    s_fov_min += s_fov_shift;
    s_fov_max += s_fov_shift;
    assert(s_fov_min>=0 && s_fov_min<=MAP_S_MAX);
    assert(s_fov_max>=0 && s_fov_max<=MAP_S_MAX);

    double car_states_s = car_states.s;
    car_states_s += s_fov_shift;

    for(int i=0; i < sensor_fusion.size(); i++){
        double s = sensor_fusion[i][5] + s_fov_shift;
        //get info of objects in FOV
        if(s>=s_fov_min && s<=s_fov_max ){
            double d = sensor_fusion[i][6];
            int lane = (int)(d/LANE_WIDTH);
            if(lane <0 || lane>2) continue; //in case some mistakes from sensor fusion outputs
            double distance2car = fabs(s - car_states_s);

            if(s >= car_states_s){// front
                //focus on the closest objects, while multiple were detected in FOV
                if(distance2car < front_distance_closest[lane]){
                    front_distance_closest[lane] = distance2car;
                    front_obj_closest[lane] = i;
                }
            }else{ // back
                if(distance2car < back_distance_closest[lane]){
                    back_distance_closest[lane] = distance2car;
                    back_obj_closest[lane] = i;
                }

            }

        }
    }
    return {front_obj_closest[0], back_obj_closest[0], front_obj_closest[1], back_obj_closest[1],
                front_obj_closest[2], back_obj_closest[2]};
}



/* @brief Set safety distance to the other cars
 * @note Other cars: on different lanes in same direction
 *
 * @param sensor_fusion
 * @param car_states
 * */
void Prediction::SetSafetyDistances(SensorFusionType const &sensor_fusion, CarStates const &car_states){
    // ego car states:
    double velocity_ego = mph_to_ms(car_states.speed);
    double decelerate_ego = CAR_MAX_ACCELERATION;
    double time_til_stop = velocity_ego/decelerate_ego;

    double front_car_velocity = GetSensorFusionVelocity(sensor_fusion, front_obj_closest[car_states.lane], CAR_MAX_SPEED);
    double distance_to_front_car = front_distance_closest[car_states.lane];

    double dv = velocity_ego - front_car_velocity;
    double time_til_collision, time_til_decelerate;
    if(dv > 0){
        time_til_collision = distance_to_front_car / dv;
        time_til_decelerate = dv / decelerate_ego;
        safety_distance = velocity_ego * time_til_decelerate + SAFETY_MARGIN_LONGITUDINAL * 2;
    } else{
        time_til_collision = INFINITY;
        time_til_decelerate = 0;
        safety_distance = SAFETY_MARGIN_LONGITUDINAL * 2;
    }

    //set saftey distance on ohter cars in different lanes
    for(int i=0; i<HIGHWAY_LANE_NUMBERS; i++){
        front_velocity[i] = GetSensorFusionVelocity(sensor_fusion, front_obj_closest[i], CAR_MAX_SPEED);
        front_safety_distance[i] = GetSafetyDistance(velocity_ego, front_velocity[i], decelerate_ego, 0.0);

        back_velocity[i] = GetSensorFusionVelocity(sensor_fusion, back_obj_closest[i], 0);
        back_safety_distance[i] = GetSafetyDistance(back_velocity[i], velocity_ego, decelerate_ego, 2.0);
    }
}

/* @brief calculate the velocity from vx and vy
 * @return velocity in Euclidean Coordinates
 * */
double Prediction::GetSensorFusionVelocity(SensorFusionType const &sensor_fusion, int idx, double default_vel)
{
  double vx, vy, vel;
  if (idx >= 0 && idx < sensor_fusion.size()) {
    vx = sensor_fusion[idx][3];
    vy = sensor_fusion[idx][4];
    vel = sqrt(vx*vx+vy*vy);
  } else {
    vel = default_vel;
  }
  return vel;
}



/* @brief calculate the safety distance
 * @param vel_back: the velocity of the car in the back
 * @param vel_front: the velocity of the car at the front
 * @param ego_decelerate: the ego car deceleration speed
 * @param time_latency: extra time margin
 *
 * */
double Prediction::GetSafetyDistance(double vel_back, double vel_front, double ego_decelerate, double time_latency)
{
  double safety_distance = SAFETY_DISTANCE_LANE_CHANGE;
  if (vel_back > vel_front) {
      double time_to_decelerate = (vel_back - vel_front) / ego_decelerate + time_latency;
      safety_distance = vel_back * time_to_decelerate + 2 * SAFETY_MARGIN_LONGITUDINAL;
  }
  safety_distance = max(safety_distance, SAFETY_DISTANCE_LANE_CHANGE);
  return safety_distance;
}

/* @brief public function, outputs safety distance
 *
 * */
double Prediction::OutputSafetyDistance() const{
    return safety_distance;
}


/*
 *
 * */
void Prediction::SetLaneInfo(SensorFusionType const &sensor_fusion, CarStates const &car_states)
{
  int car_lane = (int)(car_states.d / LANE_WIDTH);

  for (int i = 0; i < front_obj_closest.size(); i++) {

    int lane = i;
    if (front_obj_closest[i] >= 0) { // if there is a car in front
        // other lanes, below safety distance
        if (lane != car_lane && (back_distance_closest[i] <= back_safety_distance[i] || front_distance_closest[i] <= front_safety_distance[i])) {
            lane_speed[i] = 0;
            lane_free_space[i] = 0;
        }else {
            double vx = sensor_fusion[front_obj_closest[i]][3];
            double vy = sensor_fusion[front_obj_closest[i]][4];
            lane_speed[i] = sqrt(vx*vx+vy*vy);
            lane_free_space[i] = front_distance_closest[i];
        }
    }else {  // if no car in front
        // other lanes, below safety distance
        if (lane != car_lane && back_distance_closest[i] <= back_safety_distance[i]) {
            lane_speed[i] = 0;
            lane_free_space[i] = 0;
        }else {
            lane_speed[i] = CAR_MAX_SPEED_MPH;
            lane_free_space[i] = DETECT_S_FOV;

        }
    }
    cout << "Predictions::lane_speed[" << i << "]=" << lane_speed[i] << endl;
  }
}


double Prediction::OutputLaneSpeed(int lane) const {
  if (lane >= 0 && lane < HIGHWAY_LANE_NUMBERS) {
    return lane_speed[lane];
  } else {
    return 0;
  }
}

double Prediction::OutputLaneFreeSpace(int lane) const {
  if (lane >= 0 && lane < HIGHWAY_LANE_NUMBERS) {
    return lane_free_space[lane];
  } else {
    return 0;
  }
}
















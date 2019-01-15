#include "behaviour.h"

/* @brief decide the target(lane and speed) on Behaviour Layer, based on Prediction Layer and Localization layer.
 * @param sensor_fusion: input detected objects' states: [ id, x, y, vx, vy, s, d]
 * @param car_states: the autonomous vehicel (ego)
 * @param prediction: input predicted lane speed, and safety distance
 *
 * */
Behaviour::Behaviour(SensorFusionType const &sensor_fusion, CarStates const &car_states, Prediction const &prediction){
    BehaviourTarget behaviour_target;
    behaviour_target.time = 2.0;
    double car_speed_target = car_states.speed_target;
    double safety_distance = prediction.OutputSafetyDistance();

    if(car_states.emergency){
        car_speed_target = car_states.speed;
    }

    int throttle = 0; //-1 for max deceleration, 0 for constant speed, +1 for max acceleration
    bool too_close = false;
    double car_speed_target_ms = mph_to_ms(car_speed_target);
    double closest_distance = INFINITY;
    double closest_speed_ms = CAR_MAX_SPEED;

    //calaulate the reference velocity based on the front car
    for(int i=0; i<sensor_fusion.size(); i++){
        double d = sensor_fusion[i][6];
        if (d > get_d_left(car_states.lane) && d < get_d_right(car_states.lane)){
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double obj_speed = sqrt(vx*vx+vy*vy);
            double obj_s_coord = sensor_fusion[i][5];

            double relevant_distance = obj_s_coord - car_states.s;
            //if car in front and below the safety distance, slow down to the same speed.
            if((obj_s_coord > car_states.s) && relevant_distance < safety_distance){
                too_close = true;
                if(relevant_distance < closest_distance){
                    closest_distance = relevant_distance;
                    closest_speed_ms = obj_speed;
                }
            }
        }
    }

    if(too_close){
        if(car_speed_target_ms > closest_speed_ms){ // m/s
            car_speed_target -= MAX_SPEED_INC_MPH; //ego car slow down,   mph
            if(closest_distance <= 10){
                car_speed_target -= 5*MAX_SPEED_INC_MPH;
            }
        }
        //at the most, sotp the car, no backword drive, car_speed_target>0,
        car_speed_target = max(car_speed_target, 0.0);
        throttle = -1;
    }else if(car_speed_target < CAR_MAX_SPEED_MPH){
        car_speed_target += MAX_SPEED_INC_MPH;
        car_speed_target = min(car_speed_target, CAR_MAX_SPEED_MPH);//speed up until the predifined Max Speed.
        throttle = 1;
    }

    //In general case, keep the lane and speed as it is
    behaviour_target.lane = car_states.lane;
    behaviour_target.velocity = car_speed_target; //mph

    behaviour_target_list.push_back(behaviour_target);

    // Backup target ----------------------------------
    /*If it is not valid, we would have to find a collision-free
      alternative, some kind of “second best” trajectory, by slightly
      modifying T along with the coefficients of d(t) (and s(t))
      and check for collision again, and so on.
     * */
    vector<int> backup_lanes;
    switch (car_states.lane)
    {
        case 2:
            backup_lanes.push_back(1);
            break;
        case 1:
            backup_lanes.push_back(2);
            backup_lanes.push_back(0);
            break;
        case 0:
            backup_lanes.push_back(1);
            break;
        default:
            assert(1 == 0); // something went wrong
            break;
    }

    vector<double> backup_velocity;  // lower speed so far, TODO deal with other situations
    switch (throttle)
    {
        case 1:
            backup_velocity.push_back(car_speed_target - MAX_SPEED_INC_MPH);
            backup_velocity.push_back(car_speed_target - 2 * MAX_SPEED_INC_MPH);
            break;
        case 0: // already max speed
            backup_velocity.push_back(car_speed_target - MAX_SPEED_INC_MPH);
            break;
        case -1:
            // emergency breaking
            backup_velocity.push_back(car_speed_target - MAX_SPEED_INC_MPH);
            break;
        default:
            assert(1 == 0); // something went wrong
        break;
    }

    //backup 1: backup velocity on original lane
    behaviour_target.lane = car_states.lane;
    for(int i=0; i< backup_velocity.size(); i++){
        behaviour_target.velocity = backup_velocity[i];
        behaviour_target_list.push_back(behaviour_target);
    }

    //backup 2: onriginal velocity on backup lane
    behaviour_target.velocity = car_speed_target;
    for(int i=0; i< backup_lanes.size(); i++){
        behaviour_target.lane = backup_lanes[i];
        behaviour_target_list.push_back(behaviour_target);
    }

    //backup 3: backup velocity on backup lane
    for(int i=0; i<backup_velocity.size(); i++){
        behaviour_target.velocity = backup_velocity[i];
        for(int j=0; j<backup_lanes.size(); j++){
            behaviour_target.lane = backup_lanes[j];
            behaviour_target_list.push_back(behaviour_target);
        }
    }

    // backup 4: emergency
    behaviour_target.lane = car_states.lane;
    behaviour_target.velocity = prediction.OutputLaneSpeed(car_states.lane);
    behaviour_target.time = 0.0; //ASAP
    behaviour_target.acceleration = -0.8*CAR_MAX_ACCELERATION;
    behaviour_target_list.push_back(behaviour_target);
}

Behaviour::~Behaviour(){}

/* @return  behaviour_target_list: output the targets on behaviour layer
 *
 * */
vector<BehaviourTarget> Behaviour::OutputBehaviourTarget(){
    return behaviour_target_list;
}



/* @brief calculate the d coordinate for the left lane
 *
 * */
double Behaviour::get_d_left(int lane){
    double d_left = lane * LANE_WIDTH;
    return d_left;
}
/* @brief calculate the d coordinate for the right lane
 *
 * */
double Behaviour::get_d_right(int lane){
    double d_right = (lane+1) * LANE_WIDTH;
    return d_right;
}


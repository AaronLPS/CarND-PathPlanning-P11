#include "trajectory.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* @brief creaat trajectories based on behaviour targets, and find the minimum cost trajectory.
 * @param targets: behaviour planning targets
 * @param map:
 * @param car states:
 * @param previous path: will be used as start of next trajectory
 * @param prediction: objects position prediction for trajectories cost evaluation
 * */

Trajectory::Trajectory(vector<BehaviourTarget> targets, Map &map, CarStates &car, PreviousPath &previous_path, Prediction &predictions)
{
    for (size_t i = 0; i < targets.size(); i++) {//creat a trajectory for each target on behaviour layer
        TrajectoryXY trajectory;
        if (TRAJECTORY_JMT_ENABLE) {// create Jerk Minmum Trajectory
            TrajectoryJMT trajectory_jmt;
  
            // generate JMT trajectory in s-d coordinates, convert to x-y coordinates for trajectory output
            if (targets[i].time == 0){  //if in emergency
                trajectory_jmt = GenerateTrajectoryEmergency(targets[i], map, car, previous_path);
            }else{  //JMT
                trajectory_jmt = GenerateTrajectoryJMT(targets[i], map, previous_path);
            }

            trajectory = trajectory_jmt.trajectory;
            trajectories_sd_list.push_back(trajectory_jmt.path_sd);
        } else {
            // generate SPLINE trajectory in (x,y) coordinates
            trajectory = GenerateTrajectorySPLINE(targets[i], map, car, previous_path);
        }

        //calculate cost function for each trajectory
        Cost cost = Cost(trajectory, targets[i], predictions, car.lane);
        costs_.push_back(cost);
        trajectories_list.push_back(trajectory);
    }
  
    //retrieve the lowest cost trajectory
    min_cost = INFINITY;
    min_cost_index = 0;
    for (int i = 0; i < costs_.size(); i++) {
        if (costs_[i].get_cost() < min_cost) {
            min_cost = costs_[i].get_cost();
            min_cost_index = i;
        }
    }

  // enforce emergency traj: last one (in case of unavoidable collision we prefer lower speed anyways)
  if (min_cost >= WEIGHT_COST_FEASIBILITY) {
    min_cost_index = costs_.size() - 1;
    min_cost = costs_[min_cost_index].get_cost();
  }

  if (targets[min_cost_index].time == 0) {
    car.emergency = true;
  } else {
    car.emergency = false;
  }

  if (car.emergency) {
    cout << "Tajectory short distance";
  }
}




/* @brief Generate the trajectory for emergency situation
 * @note  This generator is only used in emergency situation.
 *        Thus there is no changes on d. (stay in the same lane)
 *        and  keep constant accel/decel in 2sec waypoints
 *
 * @return trajectory on s-d and x-y coordinates
 * */
TrajectoryJMT Trajectory::GenerateTrajectoryEmergency(BehaviourTarget target, Map &map, CarStates const &car, PreviousPath const &previous_path){

    TrajectoryJMT trajectory_jmt;

    TrajectoryXY previous_path_xy = previous_path.xy;
    vector<double> previous_path_x = previous_path_xy.path_x;
    vector<double> previous_path_y = previous_path_xy.path_y;

    TrajectorySD previous_path_sd = previous_path.sd;
    vector<PointC2> previous_path_s = previous_path_sd.path_s;
    vector<PointC2> previous_path_d = previous_path_sd.path_d;

    int previous_path_reused_size = previous_path.num_xy_reused;

    vector<double> new_path_x;
    vector<double> new_path_y;

    vector<PointC2> new_path_s(TRAJECTORY_WAYPOINTS_NUMBER, PointC2(0,0,0)); // initialize the waypoints on new path, based on its size
    vector<PointC2> new_path_d(TRAJECTORY_WAYPOINTS_NUMBER, PointC2(0,0,0));

    double target_velocity_ms = mph_to_ms(target.velocity);

    double s, s_dot, s_ddot;
    double d, d_dot, d_ddot;

    /* it follows the remainder of the previously calculated trajectory in each planning step
     * and therefore temporal consistency is provided.
     *
     * initialize the new trajectory on s-d coordinates
     */
    if (previous_path_reused_size > 0) {
        for (int i = 0; i < previous_path_reused_size; i++) {
            // [0]:oldest waypoint  [size()-1]: newest waypoint
            new_path_s[i] = previous_path_s[TRAJECTORY_WAYPOINTS_NUMBER - previous_path_x.size() + i];
            new_path_d[i] = previous_path_d[TRAJECTORY_WAYPOINTS_NUMBER - previous_path_x.size() + i];

            new_path_x.push_back(previous_path_x[i]);
            new_path_y.push_back(previous_path_y[i]);
        }
        // initial conditions
        s = new_path_s[previous_path_reused_size-1].f;
        s_dot = new_path_s[previous_path_reused_size-1].f_dot;
        d = new_path_d[previous_path_reused_size-1].f;
        d_dot = 0;
        d_ddot = 0;
    } else { //no previous trajectory, use car states instead
        s = car.s;
        s_dot = car.speed;
        d = car.d;
        d_dot = 0;
        d_ddot = 0;
    }
    s_ddot = target.acceleration;

    double t = DT_PER_POINT;// time interval for calculating s_dot and s
    double prev_s_dot = s_dot;
    //infer the following new trajectory
    for (int i = previous_path_reused_size; i < TRAJECTORY_WAYPOINTS_NUMBER; i++) {
        // increase/decrease speed till target velocity is reached
        s_dot += s_ddot * DT_PER_POINT;
        if ((target.acceleration > 0 && prev_s_dot <= target_velocity_ms && s_dot > target_velocity_ms) ||
            (target.acceleration < 0 && prev_s_dot >= target_velocity_ms && s_dot < target_velocity_ms)) {
            s_dot = target_velocity_ms;
        }
        s_dot = max(min(s_dot, 0.8 * CAR_MAX_SPEED), 0.0); //assert there is no driving in reverse && lower than speed limitation
        s += s_dot * DT_PER_POINT;

        prev_s_dot = s_dot;

        new_path_s[i] = PointC2(s, s_dot, s_ddot);
        new_path_d[i] = PointC2(d, d_dot, d_ddot);
        //convert to x-y coordinates
        vector<double> point_xy = map.getXYspline(s, d);

        new_path_x.push_back(point_xy[0]);
        new_path_y.push_back(point_xy[1]);

        t += DT_PER_POINT;
    }

    trajectory_jmt.trajectory = TrajectoryXY(new_path_x, new_path_y);
    trajectory_jmt.path_sd = TrajectorySD(new_path_s, new_path_d);

    return trajectory_jmt;
}

  
/* @brief Initialization for JMT trajectory generation, with the first car status
 * @param car's Frenet coordinates (s,d)
 *
 * */
TrajectoryJMT JMT_init(double car_s, double car_d)
{
    TrajectoryJMT traj_jmt;

    vector<PointC2> store_path_s(TRAJECTORY_WAYPOINTS_NUMBER, PointC2(0, 0, 0)); // 50 x {s, s_dot, s_ddot}
    vector<PointC2> store_path_d(TRAJECTORY_WAYPOINTS_NUMBER, PointC2(0, 0, 0));

    for (int i = 0; i < TRAJECTORY_WAYPOINTS_NUMBER; i++) {
        store_path_s[i] = PointC2(car_s, 0, 0);
        store_path_d[i] = PointC2(car_d, 0, 0);
    }

    traj_jmt.path_sd.path_s = store_path_s;
    traj_jmt.path_sd.path_d = store_path_d;

    return traj_jmt;
}

/* @brief Calaulate the jerk-optimal connection between a start state P0 = [p0, p˙0, p¨0]
 * and an end state P1 = [p1, p˙1, p¨1] within the time interval T := t1 − t0
 *
 * @note: quintic polynomials are the jerk-optimal connection in a one-dimensional problem
 *        Reference: Moritz Werling, Julius Ziegler, So¨ren Kammel, and Sebastian Thrun.
 *                   Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame
 *
 * @param start: car start state
 * @param end: car end state
 * @param T: time interval (in sec) between start and end
 *
 * Output: coefficients fo the quintic polynomial, an 6-length array,
 * s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
 *
 * e.g
 * > JMT( [0, 10, 0], [10, 10, 0], 1)
 * [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
 *
 *
 * */
vector<double> Trajectory::JMT(vector< double> start, vector <double> end, double T){

    MatrixXd A(3,3);
    VectorXd b(3);
    VectorXd x(3);

    A <<   pow(T,3),    pow(T,4),    pow(T,5),
         3*pow(T,2),  4*pow(T,3),  5*pow(T,4),
                6*T, 12*pow(T,2), 20*pow(T,3);

    b << end[0] - (start[0] + start[1]*T + 0.5*start[2]*T*T), 
         end[1] - (start[1] + start[2]*T), 
         end[2] - start[2];

    x = A.inverse() * b;

    return {start[0], start[1], start[2]/2, x[0], x[1], x[2]};
}

/* @brief Use JMT to generate trajectory on both (s,d) and (x,y) coordinates, based on behaviour target and previous path.
 * @note Reference: Moritz Werling, Julius Ziegler, So¨ren Kammel, and Sebastian Thrun.
 *                   Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame
 *
 * @param BehaviourTarget: shows the end states of the trajectory
 * @param Map: convert (s,d) coordinates to (x,y)
 * @param PreviousPath: shows the start states of the trajectory
 *
 * @return TrajectoryJMT: trajectory on both (s,d) and (x,y) coordinates
 * */
TrajectoryJMT Trajectory::GenerateTrajectoryJMT(BehaviourTarget target, Map &map, PreviousPath const &previous_path)
{
    TrajectoryJMT traj_jmt;

    TrajectoryXY previous_path_xy = previous_path.xy;
    vector<double> previous_path_x = previous_path_xy.path_x;
    vector<double> previous_path_y = previous_path_xy.path_y;

    int previous_path_reused_size = previous_path.num_xy_reused;

    TrajectorySD previous_path_sd = previous_path.sd;
    vector<PointC2> prev_path_s = previous_path_sd.path_s;
    vector<PointC2> prev_path_d = previous_path_sd.path_d;

    vector<PointC2> new_path_s(TRAJECTORY_WAYPOINTS_NUMBER, PointC2(0,0,0));
    vector<PointC2> new_path_d(TRAJECTORY_WAYPOINTS_NUMBER, PointC2(0,0,0));

    int last_point;
    if (REACTION_LATENCY_WAYPOINTS < TRAJECTORY_WAYPOINTS_NUMBER) {
        last_point = TRAJECTORY_WAYPOINTS_NUMBER - previous_path_x.size() + previous_path_reused_size - 1; // TODO Evaluate
    } else {
        last_point = TRAJECTORY_WAYPOINTS_NUMBER - 1;
    }
    //Time interval T:
    double T = target.time; // 2 sec
    //Start states (s,d): get from the previous path
    double begin_s, begin_s_dot, begin_s_ddot;
    double begin_d, begin_d_dot, begin_d_ddot;
    
    begin_s = prev_path_s[last_point].f;
    begin_s_dot  = prev_path_s[last_point].f_dot;
    begin_s_ddot = prev_path_s[last_point].f_ddot;
    
    begin_d      = prev_path_d[last_point].f;
    begin_d_dot  = prev_path_d[last_point].f_dot;
    begin_d_ddot = prev_path_d[last_point].f_ddot;
    
    //End states (s,d): target
    double end_s, end_s_dot, end_s_ddot;
    double end_d, end_d_dot, end_d_ddot;

    /* - At higher speeds, d(t) and s(t) can be chosen independently
     * - At extreme low speeds, this strategy disregards the non-holonomic
     *   property of the car, so that the majority of the trajectories has to be rejected
     *   due to invalid curvatures
     *
     * - Thus, the behavioral layer can switch below a certain velocity threshold to a slightly
     *   different trajectory mode generating the lateral trajectory in dependence on the
     *   longitudinal movement
     * */
    if (target.velocity <= 10) { // Low Speed Trajectories;  mph
        end_s_ddot = 0;
        end_s_dot  = mph_to_ms(target.velocity);

        //for speed limitation m/s
        end_s_dot = min(end_s_dot, begin_s_dot + 10 * MAX_SPEED_INC);//10*0.02s = 2 sec
        end_s_dot = min(end_s_dot, CAR_MAX_SPEED);
        end_s_dot = max(end_s_dot, begin_s_dot - 10 * MAX_SPEED_INC);

        // end_s      = begin_s + 2 * end_s_dot * T;  //TODO evaluate
        end_s      = begin_s + end_s_dot * T;

        end_d_ddot =  0;
        end_d_dot  =  0;
        end_d      = begin_d;

    } else { // High Speed Trajectories
        //s
        end_s_ddot = 0;
        end_s_dot = mph_to_ms(target.velocity);

        //for speed limitation m/s
        end_s_dot = min(end_s_dot, begin_s_dot + 10 * MAX_SPEED_INC);
        end_s_dot = min(end_s_dot, CAR_MAX_SPEED);
        end_s_dot = max(end_s_dot, begin_s_dot - 10 * MAX_SPEED_INC);

        end_s = begin_s + end_s_dot * T;

        end_d_ddot = 0;
        end_d_dot  = 0;
        end_d      = get_dcenter(target.lane);
    }

    vector<double> start_s_states = { begin_s, begin_s_dot, begin_s_ddot};
    vector<double> end_s_states = { end_s, end_s_dot, end_s_ddot};

    vector<double> start_d_states = { begin_d, begin_d_dot, begin_d_ddot };
    vector<double> end_d_states = { end_d, end_d_dot, end_d_ddot};

    //Generate Trajectory (s,d)
    vector<double> poly_s = JMT(start_s_states, end_s_states, T); //generate longitudinal movement
    vector<double> poly_d = JMT(start_d_states, end_d_states, T); //generate lateral movement

    //convert from (s,d) to (x,y) coordinates
    vector<double> new_path_x;
    vector<double> new_path_y;
    //it follows the remainder of the previously calculated trajectory in each planning step
    for (int i = 0; i < previous_path_reused_size; i++) {
        new_path_s[i] = prev_path_s[TRAJECTORY_WAYPOINTS_NUMBER - previous_path_x.size() + i];
        new_path_d[i] = prev_path_d[TRAJECTORY_WAYPOINTS_NUMBER - previous_path_x.size() + i];

        new_path_x.push_back(previous_path_x[i]);
        new_path_y.push_back(previous_path_y[i]);
    }

    //infer the following trajectory
    double t = DT_PER_POINT;
    for (int i = previous_path_reused_size; i < TRAJECTORY_WAYPOINTS_NUMBER; i++) {
        double s = calculate_polynomial(poly_s, t);
        double s_dot = calculate_polynomial_dot(poly_s, t);
        double s_ddot = calculate_polynomial_ddot(poly_s, t);

        double d = calculate_polynomial(poly_d, t);
        double d_dot = calculate_polynomial_dot(poly_d, t);
        double d_ddot = calculate_polynomial_ddot(poly_d, t);

        new_path_s[i] = PointC2(s, s_dot, s_ddot);
        new_path_d[i] = PointC2(d, d_dot, d_ddot);

        vector<double> point_xy = map.getXYspline(s, d);

        new_path_x.push_back(point_xy[0]);
        new_path_y.push_back(point_xy[1]);

        t += DT_PER_POINT;
    }

  traj_jmt.trajectory = TrajectoryXY(new_path_x, new_path_y);
  traj_jmt.path_sd = TrajectorySD(new_path_s, new_path_d);

  return traj_jmt;
}



/* @brief calculate the value of the polynomial at time t: F(t)
 * F(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
 * @param c: input coefficients of polynomial
 * @return the value F(t)
 * */
double Trajectory::calculate_polynomial(vector<double> c, double t) {
  double res = 0.0;
  for (size_t i = 0; i < c.size(); i++) {
    res += c[i] * pow(t, i);
  }
  return res;
}

/* @brief calculate 1st derivative of a polynomial at time t: F'(t)
 *
 * */
double Trajectory::calculate_polynomial_dot(vector<double> c, double t) {
  double res = 0.0;
  for (size_t i = 1; i < c.size(); ++i) {
    res += i * c[i] * pow(t, i-1);
  }
  return res;
}

/* @brief calculate 2st derivative of a polynomial at time t: F"(t)
 *
 * */
double Trajectory::calculate_polynomial_ddot(vector<double> c, double t) {
  double res = 0.0;
  for (size_t i = 2; i < c.size(); ++i) {
    res += i * (i-1) * c[i] * pow(t, i-2);
  }
  return res;
}




TrajectoryXY Trajectory::GenerateTrajectorySPLINE(BehaviourTarget target, Map &map, CarStates const &car, PreviousPath const &previous_path)
{
    TrajectoryXY previous_path_xy = previous_path.xy;
    int previous_path_size = previous_path.num_xy_reused;

    vector<double> previous_path_x = previous_path_xy.path_x;
    vector<double> previous_path_y = previous_path_xy.path_y;

    vector<double> ptsx;
    vector<double> ptsy;

    double ref_x = car.x;
    double ref_y = car.y;
    double ref_yaw = deg2rad(car.yaw);

    if (previous_path_size < 2) {
        double prev_car_x = car.x - cos(car.yaw);
        double prev_car_y = car.y - sin(car.yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car.x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car.y);
    } else {
        ref_x = previous_path_x[previous_path_size-1];
        ref_y = previous_path_y[previous_path_size-1];

        double ref_x_prev = previous_path_x[previous_path_size-2];
        double ref_y_prev = previous_path_y[previous_path_size-2];
        ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    vector<double> next_wp0 = map.getXYspline(car.s+30, get_dcenter(target.lane));
    vector<double> next_wp1 = map.getXYspline(car.s+60, get_dcenter(target.lane));
    vector<double> next_wp2 = map.getXYspline(car.s+90, get_dcenter(target.lane));


    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);
  
  
    for (int i = 0; i < ptsx.size(); i++) {
        // shift car reference angle to 0 degrees
        // transformation to local car's coordinates (cf MPC)
        // last point of previous path at origin and its angle at zero degree

        // shift and rotation
        double shift_x = ptsx[i]-ref_x;
        double shift_y = ptsy[i]-ref_y;

        ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
        ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
    }
  
    tk::spline spl;
    spl.set_points(ptsx, ptsy);
  
    vector<double> next_x_vals;
    vector<double> next_y_vals;
  
    for (int i = 0; i < previous_path_size; i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }
  
    // Calculate how to break up spline points so that we travel at our desired reference velocity
    double target_x = 30.0;
    double target_y = spl(target_x);
    double target_dist = sqrt(target_x*target_x + target_y*target_y);

    double x_add_on = 0;
  
    // fill up the rest of our path planner after filing it with previous points
    // here we will always output 50 points
    for (int i = 1; i <= TRAJECTORY_WAYPOINTS_NUMBER - previous_path_size; i++) {
        double N = (target_dist / (DT_PER_POINT * mph_to_ms(target.velocity)));
        double x_point = x_add_on + target_x/N;
        double y_point = spl(x_point);

        x_add_on = x_point;

        double x_ref = x_point; // x_ref IS NOT ref_x !!!
        double y_ref = y_point;

        // rotate back to normal after rotating it earlier
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }

    return TrajectoryXY(next_x_vals, next_y_vals);
}

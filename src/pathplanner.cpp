/*
 * Implements a path planning algorithm to control the car
 *
 * Uses a finite state machine with cost functions to choose the best
 * trajectory.
 *
 */

#include "pathplanner.h"
#include <iostream>

PathPlanner::PathPlanner(vector<double> map_waypoints_x,
                         vector<double> map_waypoints_y,
                         vector<double> map_waypoints_s,
                         double horizon) {
  // Initialisation
  _map_waypoints_x = map_waypoints_x;
  _map_waypoints_y = map_waypoints_y;
  _map_waypoints_s = map_waypoints_s;
  _horizon = horizon;

  // Matrix used of calculation of jerk free paths
  double T = _horizon;
  double T2 = T * T;
  double T3 = T2 * T;
  double T4 = T3 * T;
  double T5 = T4 * T;
  _A << T3, T4, T5,
        3 * T2, 4 * T3, 5 * T4,
        6 * T, 12 * T2, 20 * T3;
}

vector<vector<double>> PathPlanner::update(double car_x, double car_y,
                                           double car_yaw,
                                           double car_s, double car_d,
                                           double car_v,
                                           vector<double> previous_path_x,
                                           vector<double> previous_path_y) {
  // Create list of possible new states
  vector<string> new_states = valid_states;
  // Compute a trajectory for each new state
  vector<vector<vector<double>>> candidate_trajectories;
  vector<PathPlanner::Target> new_targets;
  for (int s = 0; s < new_states.size(); s++) {
    PathPlanner::Target new_target = _action(new_states[s]);
    vector<vector<double>> candidate = _trajectory(car_x, car_y, car_yaw,
                                        car_s, car_d, car_v,
                                        previous_path_x, previous_path_y,
                                        new_target.speed, new_target.lane);
    new_targets.push_back(new_target);
    candidate_trajectories.push_back(candidate);
  }
  // Compute a cost for each new trajectory
  vector<double> costs;
  for (int s = 0; s < new_states.size(); s++) {
    costs.push_back(_cost(new_targets[s], candidate_trajectories[s]));
  }
  // Choose the trajectory with the lowest cost
  double min_cost = 100.0;
  int min_cost_index = 0;
  for (int s = 0; s < new_states.size(); s++) {
    if (costs[s] < min_cost) {
      min_cost = costs[s];
      min_cost_index = s;
    }
  }
  cout << "Action: " << new_states[min_cost_index] << endl;
  // Update target
  target = new_targets[min_cost_index];
  return candidate_trajectories[min_cost_index];
}

PathPlanner::Target PathPlanner::_action(string state) {
  PathPlanner::Target new_target;
  if (state == "KL") {
    // Keep lane and speed
    new_target = target;
  } else if (state == "ACCEL") {
    new_target = {target.speed + 0.1, target.lane};
  } else if (state == "DECEL") {
    new_target = {target.speed - 0.1, target.lane};
  }
  return new_target;
}

double PathPlanner::_cost(PathPlanner::Target new_target,
                          vector<vector<double>> trajectory) {
  // Cost function for candidate trajectory
  return abs(new_target.speed - speed_limit);
}

vector<vector<double>> PathPlanner::_trajectory(double car_x, double car_y,
                                   double car_yaw,
                                   double car_s, double car_d, double car_v,
                                   vector<double> previous_path_x,
                                   vector<double> previous_path_y,
                                   double new_speed, int new_lane) {
  // Build a jerk free trajectory to the new lane and speed

  // Initial conditons: estimate velocity and acceleration by finite differences
  double x_i = car_x;
  double y_i = car_y;
  double x_i_dot, x_i_dotdot, y_i_dot, y_i_dotdot;
  if (previous_path_x.size() < 3) {
    // Probably just started, so velocity and acceleration are zero
    x_i_dot = 0;
    x_i_dotdot = 0;
    y_i_dot = 0;
    y_i_dotdot = 0;
  } else {
    x_i_dot = (previous_path_x[1] - previous_path_x[0]) / dt;
    x_i_dotdot = (previous_path_x[2] - 2 * previous_path_x[1] + previous_path_x[0]) / (dt * dt);
    y_i_dot = (previous_path_y[1] - previous_path_y[0]) / dt;
    y_i_dotdot = (previous_path_y[2] - 2 * previous_path_y[1] + previous_path_y[0]) / (dt * dt);
  }
  // Final conditions (steady speed at centre of target lane)
  double x_f_dotdot = 0;
  double y_f_dotdot = 0;
  cout << "Speeds: " << car_v << ", " << new_speed << endl;
  vector<double> xy_f = getXY(car_s + new_speed * _horizon,
                              new_lane * 4.0 + 2.0, _map_waypoints_s,
                              _map_waypoints_x, _map_waypoints_y);
  // Perturbed goal to estimate final heading
  vector<double> xy_n = getXY(car_s + new_speed * _horizon + 10.0,
                              new_lane * 4.0 + 2.0, _map_waypoints_s,
                              _map_waypoints_x, _map_waypoints_y);
  double yaw_f = atan2(xy_n[0] - xy_f[0], xy_n[1] - xy_f[1]);
  double x_f = xy_f[0];
  double y_f = xy_f[1];
  double x_f_dot = new_speed * sin(yaw_f);
  double y_f_dot = new_speed * cos(yaw_f);

  VectorXd b = VectorXd(3);
  // Solve for coefficients to compute x
  b << x_f - (x_i + x_i_dot * _horizon + 0.5 * x_i_dotdot * pow(_horizon, 2)),
       x_f_dot - (x_i_dot + x_i_dotdot * _horizon),
       x_f_dotdot - x_i_dotdot;
  VectorXd alpha_x = _A.colPivHouseholderQr().solve(b);
  // Solve for coefficients to compute y
  b << y_f - (y_i + y_i_dot * _horizon + 0.5 * y_i_dotdot * pow(_horizon, 2)),
       y_f_dot - (y_i_dot + y_i_dotdot * _horizon),
       y_f_dotdot - y_i_dotdot;
  VectorXd alpha_y = _A.colPivHouseholderQr().solve(b);

  // Now compute a trajectory of coordinates along this path
  vector<double> next_x;
  vector<double> next_y;
  for (int i = 0; i < _horizon / dt; i++) {
    double t = i * dt;
    double x = x_i + x_i_dot * t + 0.5 * x_i_dotdot * pow(t, 2) +
               alpha_x[0] * pow(t, 3) + alpha_x[1] * pow(t, 4) +
               alpha_x[2] * pow(t, 5);
    double y = y_i + y_i_dot * t + 0.5 * y_i_dotdot * pow(t, 2) +
               alpha_y[0] * pow(t, 3) + alpha_y[1] * pow(t, 4) +
               alpha_y[2] * pow(t, 5);
    next_x.push_back(x);
    next_y.push_back(y);
  }

  return {next_x, next_y};
}


// Utility functions copied from main.cpp template

constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int ClosestWaypoint(double x, double y,
                    vector<double> maps_x, vector<double> maps_y) {

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x, y, map_x, map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta,
                 vector<double> maps_x, vector<double> maps_y) {

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y - y), (map_x - x));

	double angle = abs(theta - heading);

	if(angle > pi() / 4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
                         vector<double> maps_x, vector<double> maps_y) {
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp - 1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size() - 1;
	}

	double n_x = maps_x[next_wp] - maps_x[prev_wp];
	double n_y = maps_y[next_wp] - maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
	double proj_x = proj_norm * n_x;
	double proj_y = proj_norm * n_y;

	double frenet_d = distance(x_x, x_y, proj_x, proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000 - maps_x[prev_wp];
	double center_y = 2000 - maps_y[prev_wp];
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, proj_x, proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
	}

	frenet_s += distance(0, 0, proj_x, proj_y);

	return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s,
                     vector<double> maps_x, vector<double> maps_y) {
	int prev_wp = -1;

	while(s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp + 1) % maps_x.size();

	double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
                         (maps_x[wp2] - maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
	double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

	double perp_heading = heading-pi() / 2;

	double x = seg_x + d * cos(perp_heading);
	double y = seg_y + d * sin(perp_heading);

	return {x,y};
}

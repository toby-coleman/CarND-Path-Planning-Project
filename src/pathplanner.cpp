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
}

vector<vector<double>> PathPlanner::update(double car_x_, double car_y_,
                                        double car_yaw_,
                                        double car_s_, double car_d_,
                                        double car_v_,
                                        vector<double> previous_path_x,
                                        vector<double> previous_path_y,
                                        vector<vector<double>> sensor_fusion) {
  // Store car's current position, yaw, speed
  car_x = car_x_;
  car_y = car_y_;
  car_yaw = deg2rad(car_yaw_);
  car_s = car_s_;
  car_d = car_d_;
  car_v = car_v_ * 0.44704; // Convert to mph
  // Predict where all of the other cars will go
  vector<vector<vector<double>>> predictions = _predict(sensor_fusion);

  // Create list of possible new states
  vector<string> new_states = valid_states();
  // Compute a trajectory for each new state
  vector<vector<vector<double>>> candidate_trajectories;
  vector<PathPlanner::Target> new_targets;
  for (int s = 0; s < new_states.size(); s++) {
    PathPlanner::Target new_target = _action(new_states[s]);
    vector<vector<double>> candidate = _trajectory(previous_path_x,
                                        previous_path_y,
                                        new_target.speed, new_target.lane);
    new_targets.push_back(new_target);
    candidate_trajectories.push_back(candidate);
  }
  // Compute a cost for each new trajectory
  vector<double> costs;
  for (int s = 0; s < new_states.size(); s++) {
    costs.push_back(_cost(new_targets[s],
                    candidate_trajectories[s], predictions));
  }
  // Choose the trajectory with the lowest cost
  double min_cost = 1000000.0;
  int min_cost_index = 0;
  for (int s = 0; s < new_states.size(); s++) {
    cout.precision(4);
    cout << new_states[s] << ": " << costs[s] << ", ";
    if (costs[s] < min_cost) {
      min_cost = costs[s];
      min_cost_index = s;
    }
  }
  cout << endl;
  cout << "Action: " << new_states[min_cost_index] << endl;
  // Print info on closest car under this trajectory
  vector<double> close = _closest_car(candidate_trajectories[min_cost_index],
                                      predictions);
  cout << "Closest obstacles in [s, d, cartesian]: " << close[0] << ", "
       << close[1] << ", " << close[2] << endl;
  // Update target
  target = new_targets[min_cost_index];
  cout << "Target speed: " << target.speed << ", lane: " << target.lane << endl;
  // Update state
  car_state = new_states[min_cost_index];

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
  } else if (state == "LEFT") {
    new_target = {target.speed, target.lane - 1};
  } else if (state == "RIGHT") {
    new_target = {target.speed, target.lane + 1};
  }
  return new_target;
}

vector<string> PathPlanner::valid_states() {
  // Returns valid actions
  vector<string> valid = _all_states;
  if (car_state == "ACCEL") {
    // Remove DECEL
    valid.erase(remove(valid.begin(), valid.end(), "DECEL"), valid.end());
  } else if (car_state == "DECEL") {
    // Remove ACCEL
    valid.erase(remove(valid.begin(), valid.end(), "ACCEL"), valid.end());
  }
  if (target.lane == 0) {
    // Remove LEFT (already in left-most lane)
    valid.erase(remove(valid.begin(), valid.end(), "LEFT"), valid.end());
  } else if (target.lane == 2) {
    // Remove RIGHT (already in left-most lane)
    valid.erase(remove(valid.begin(), valid.end(), "RIGHT"), valid.end());
  }

  return valid;
}

double PathPlanner::_cost(PathPlanner::Target new_target,
                          vector<vector<double>> trajectory,
                          vector<vector<vector<double>>> predictions) {
  // Cost function for candidate trajectory
  double cost = 0;

  double s_f = getFrenet(trajectory[0].back(), trajectory[1].back(),
                         car_yaw, _map_waypoints_x, _map_waypoints_y)[0];

    for (int i = 1; i < trajectory[0].size(); i++) {
    // Compute speed
    double speed = distance(trajectory[0][i], trajectory[1][i],
                            trajectory[0][i - 1], trajectory[1][i - 1]) / dt;
    // Make speeds in the wrong direction negative
    if (s_f < car_s) {
      speed = -speed;
    }
    // Low speed
    cost += 0.1 * sigmoid(5.0 - speed);
    // High speed
    cost += sigmoid(10 * (speed - speed_limit + 1.0));
  }
  // Closest vehicles (avoid collisions and keep following distance)
  vector<double> closest_sd = _closest_car(trajectory, predictions);
  cost += trajectory.size() * (5.0 * sigmoid(6.0 - closest_sd[0]) +
                               50.0 * sigmoid(10 * (2.5 - closest_sd[0])));
  cost += 30 * trajectory.size() * sigmoid(20 * (2.0 - closest_sd[1]));
  // Use cartesian distance to penalise collisions
  cost += 1000 * sigmoid(15 * (2.5 - closest_sd[2]));
  // Lane change penalty
  if (target.lane != new_target.lane) {
    cost += 50;
  }
  return cost;
}

vector<vector<double>> PathPlanner::_trajectory(vector<double> previous_path_x,
                                   vector<double> previous_path_y,
                                   double new_speed, int new_lane) {
  // Build a spline trajectory to the new lane and speed

  // Points to contain new trajectory
  vector<double> next_x;
  vector<double> next_y;

  // Set up spline
  tk::spline s;
  vector<double> spline_x, spline_y;
  if (previous_path_x.size() < 3) {
    // Probably just started, so just put in current location
    spline_x.push_back(car_x);
    spline_y.push_back(car_y);
    next_x.push_back(car_x);
    next_y.push_back(car_y);
  } else {
    // Use previous path points as start of spline
    for (int i = 0; i <  3; i++) {
      spline_x.push_back(previous_path_x[i]);
      spline_y.push_back(previous_path_y[i]);
      next_x.push_back(previous_path_x[i]);
      next_y.push_back(previous_path_y[i]);
    }
  }
  // Position after manoeuvre centre of new lane
  double d_target = new_lane * 4.0 + 2.0;
  double s_target = car_s + new_speed * _t_manoeuvre;
  vector<double> xy_t = getXY(s_target,
                              d_target, _map_waypoints_s,
                              _map_waypoints_x, _map_waypoints_y);
  spline_x.push_back(xy_t[0]);
  spline_y.push_back(xy_t[1]);
  // Extra position a bit furher ahead
  vector<double> xy_f = getXY(s_target + 10.0,
                              d_target, _map_waypoints_s,
                              _map_waypoints_x, _map_waypoints_y);
  spline_x.push_back(xy_f[0]);
  spline_y.push_back(xy_f[1]);

  // Transform coordinate relative to car to prevent spline fitting errors
  double ref_x = spline_x[0];
  double ref_y = spline_y[0];
  for (int i = 0; i < spline_x.size(); i++) {
    double shift_x = spline_x[i] - ref_x;
    double shift_y = spline_y[i] - ref_y;
    spline_x[i] = (shift_x * cos(-car_yaw) - shift_y * sin(-car_yaw));
    spline_y[i] = (shift_x * sin(-car_yaw) + shift_y * cos(-car_yaw));
  }

  // Now compute a trajectory of coordinates along this spline
  s.set_points(spline_x, spline_y);
  // Transform last available point in trajectory
  double previous_x = ((next_x.back() - ref_x) * cos(-car_yaw) -
                       (next_y.back() - ref_y) * sin(-car_yaw));
  // Build new trajectory points from spline
  for (int i = 0; i < _horizon / dt; i++) {
    double new_x = previous_x + new_speed * dt;
    previous_x = new_x;
    double new_y = s(new_x);
    // Restore final point to global coordinate system
    double shift_x = new_x;
    double shift_y = new_y;
    new_x = shift_x * cos(car_yaw) - shift_y * sin(car_yaw) + ref_x;
    new_y = shift_x * sin(car_yaw) + shift_y * cos(car_yaw) + ref_y;
    next_x.push_back(new_x);
    next_y.push_back(new_y);
  }

  return {next_x, next_y};
}

vector<vector<vector<double>>> PathPlanner::_predict(
                                      vector<vector<double>> sensor_fusion) {
  // Predict the trajectories of all other vehicles, assuming they move at
  // constant velocity

  vector<vector<vector<double>>> predictions;

  // Loop over each vehicle in sensor fusion data
  for (int v = 0; v < sensor_fusion.size(); v++) {
    double x = sensor_fusion[v][1];
    double y = sensor_fusion[v][2];
    double vx = sensor_fusion[v][3];
    double vy = sensor_fusion[v][4];
    vector<double> next_x;
    vector<double> next_y;
    // Make a prediction
    for (int i = 0; i < _horizon / dt; i++) {
      double t = i * dt;
      next_x.push_back(x + vx * t);
      next_y.push_back(y + vy * t);
    }
    predictions.push_back({next_x, next_y});
  }

  return predictions;
}

vector<double> PathPlanner::_closest_car(vector<vector<double>> trajectory,
                                 vector<vector<vector<double>>> predictions) {
  // Compute distance to the other vehicles along the trajectory and return
  // the closest s and d values, along with closest cartiesian distance
  double closest_s = 10000; // large number
  double closest_d = 10000;
  double closest = 10000;

  // Compute heading of the other vehicles
  vector<double> headings;
  for (int v = 0; v < predictions.size(); v++) {
    double heading = atan2(predictions[v][0][1] - predictions[v][0][0],
                           predictions[v][1][1] - predictions[v][1][0]);
    headings.push_back(heading);
  }
  // Loop over time
  for (int i = 0; i < trajectory[0].size(); i++) {
    // Our car location
    double x = trajectory[0][i];
    double y = trajectory[1][i];
    vector<double> sd = getFrenet(x, y,
                                  car_yaw, _map_waypoints_x, _map_waypoints_y);
    // Loop over other vehicles
    for (int v = 0; v < predictions.size(); v++) {
      double other_x = predictions[v][0][i];
      double other_y = predictions[v][1][i];
      double dist = distance(x, y, other_x, other_y);
      vector<double> other_sd = getFrenet(other_x, other_y,
                              headings[v], _map_waypoints_x, _map_waypoints_y);
      // Save closest pass in s and d directions
      if ((abs(other_sd[0] - sd[0]) < closest_s) &&
          (abs(other_sd[1] - sd[1]) < 1.0)) {
        // Closest s distance for cars in the same lane
        closest_s = abs(other_sd[0] - sd[0]);
      }
      if ((abs(other_sd[1] - sd[1]) < closest_d) &&
          (abs(other_sd[0] - sd[0]) < 5.0)) {
        // Closest d distance for cars alongside us (nearby in s)
        closest_d = abs(other_sd[1] - sd[1]);
      }
      if (dist < closest) {
        // Closest cartesian distance
        closest = dist;
      }
    }
  }
  return {closest_s, closest_d, closest};
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
	int next_wp = NextWaypoint(x,y, theta, maps_x, maps_y);

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

double sigmoid(double x) {
  return 1.0 / (1.0 + exp(-x));
}

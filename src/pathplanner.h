#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <vector>
#include <string>
#include <map>
#include <cmath>
#include "spline.h"

using namespace std;

class PathPlanner {
  struct Target {
    double speed;
    int lane;
  };
private:
  // Map
  vector<double> _map_waypoints_x, _map_waypoints_y, _map_waypoints_s;
  // Horizon (how far the points should be mapped into the future)
  double _horizon;
  // Manoeuvring time (used for path planning)
  double _t_manoeuvre = 2.0;
  // Max s of track
  double _max_s = 6945.554;
  // Last lane change in xy coordinates
  vector<double> _xy_last_lane = {0, 0};
  // All possible states in order of preference given equal cost
  vector<string> _all_states = {"KL", "DECEL+", "DECEL", "ACCEL",
                                "LEFT", "RIGHT"};
  // Debug mode
  bool _debug = false;

  // Converts a state into a new speed and lane
  Target _action(string state);
  // Generates a candidate trajectory
  vector<vector<double>> _trajectory(vector<double> previous_path_x,
                                     vector<double> previous_path_y,
                                     double new_speed, int new_lane);
  // Cost function
  double _cost(Target new_target, vector<vector<double>> trajectory,
               vector<vector<vector<double>>> predictions);
  // Predict function
  vector<vector<vector<double>>> _predict(vector<vector<double>> sensor_fusion);
  // Closest car distance {sf, sb, dl, dr}
  vector<double> _closest_car(vector<vector<double>> trajectory,
                              vector<vector<vector<double>>> predictions,
                              int i);
  // Closest car (cartesian distance over whole trajectory)
  double _closest_cartesian(vector<vector<double>> trajectory,
                            vector<vector<vector<double>>> predictions);
public:
  // Speed limit
  double speed_limit = 50 * 0.44704; // 50 mph
  // Target speed and lane
  Target target = {0.0, 1};
  // Timestep
  double dt = 0.02;
  // Car state
  string car_state = "KL";
  // Current position, yaw, velocity
  double car_x;
  double car_y;
  double car_yaw;
  double car_s;
  double car_d;
  double car_v;

  // Constructor
  PathPlanner(vector<double> map_waypoints_x,
    vector<double> map_waypoints_y,
    vector<double> map_waypoints_s,
    double horizon, bool debug=false);

  // Valid states
  vector<string> valid_states();

  // Call this to plan the path and return a trajectory
  vector<vector<double>> update(double car_x_, double car_y_,
                                double car_yaw_,
                                double car_s_, double car_d_,
                                double car_v_,
                                vector<double> previous_path_x,
                                vector<double> previous_path_y,
                                vector<vector<double>> sensor_fusion);
};


// Utility functions copied from main.cpp template

constexpr double pi();

double deg2rad(double x);

double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y,
                    vector<double> maps_x, vector<double> maps_y);

int NextWaypoint(double x, double y, double theta,
                 vector<double> maps_x, vector<double> maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
                         vector<double> maps_x, vector<double> maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s,
                     vector<double> maps_x, vector<double> maps_y);

// Additional utility functions

double sigmoid(double x);

#endif /* PATHPLANNER_H */

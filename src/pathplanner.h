#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <vector>
#include <string>
#include <map>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class PathPlanner {
  struct Target {
    double speed;
    int lane;
  };
private:
  // Map
  vector<double> _map_waypoints_x, _map_waypoints_y, _map_waypoints_s;
  // Horizon (time into the future for each path)
  double _horizon;
  // Matrix to use for computing paths
  MatrixXd _A = MatrixXd(3, 3);

  // Converts a state into a new speed and lane
  Target _action(string state);
  // Generates a candidate trajectory
  vector<vector<double>> _trajectory(double car_x, double car_y, double car_yaw,
                                     double car_s, double car_d, double car_v,
                                     vector<double> previous_path_x,
                                     vector<double> previous_path_y,
                                     double new_speed, int new_lane);
  // Cost function
  double _cost(Target new_target, vector<vector<double>> trajectory);
public:
  // Speed limit
  double speed_limit = 48 * 0.44704; // 50 mph
  // Target speed and lane
  Target target = {0.0, 1};
  // Timestep
  double dt = 0.02;
  // Valid states
  vector<string> valid_states = {"KL", "ACCEL", "DECEL"};
  // Car state
  string car_state = "KL";

  // Constructor
  PathPlanner(vector<double> map_waypoints_x,
    vector<double> map_waypoints_y,
    vector<double> map_waypoints_s,
    double horizon);

  // Call this to plan the path and return a trajectory
  vector<vector<double>> update(double car_x, double car_y,
                                double car_yaw,
                                double car_s, double car_d,
                                double car_v,
                                vector<double> previous_path_x,
                                vector<double> previous_path_y);
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


#endif /* PATHPLANNER_H */

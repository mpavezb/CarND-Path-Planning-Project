#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <iostream>
#include <string>
#include <vector>

/**
 * Using SI units. Non SI data is converted during deserialization.
 */

namespace udacity {

struct Point {
  double x;
  double y;
  Point() : x(.0), y(.0) {}
  Point(double x, double y) : x(x), y(y) {}
};

std::ostream& operator<<(std::ostream& os, const Point& p) {
  return os << "(" << p.x << "," << p.y << ")";
}

typedef std::vector<Point> Path;

struct Parameters {
  // map
  std::string map_filename{"../data/highway_map.csv"};
  float max_s = 6945.554F;
  float lane_width{4.0F};
  float speed_limit{50.0 / 2.237F};

  // lifecycle
  float time_step_{0.02F};

  // target
  std::uint8_t goal_lane_id{1U};  // 0=left, 1=middle, 2=right
  float goal_s{max_s};

  // control
  float desired_speed{49.5F / 2.237F};
  float min_distance_to_vehicle{30.0F};
  float lane_speed_vehicle_distance{100.0F};
  float lane_change_gap_ahead_length{30.0F};
  float lane_change_gap_behind_length{10.0F};

  // trajectory generation
  int path_size_{50};
  int n_anchors_{5};
  float look_ahead_distance_{50.0F};
  float acceleration{0.1F};
  float deceleration{0.2F};
};

struct Map {
  double max_s{0.0};
  std::vector<double> waypoints_x;
  std::vector<double> waypoints_y;
  std::vector<double> waypoints_s;
  std::vector<double> waypoints_dx;
  std::vector<double> waypoints_dy;
};

struct FusedObject {
  double id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
};
typedef std::vector<FusedObject> FusedObjects;

struct TelemetryPacket {
  // Main car's localization Data
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;

  // A list of all other cars on the same side the road.
  FusedObjects sensor_fusion;

  // Unvisited points from previous trajectory
  Path last_path;

  // Previous path's end s and d values
  double end_path_s;
  double end_path_d;
};

struct Vehicle {
  int id;
  double s;
  double d;
  double speed;
  double lane_id;
  double distance;  // abs
  double predicted_s;
  double predicted_distance;  // abs
  bool is_ahead;
  bool is_behind;
  bool is_near;
};
typedef std::vector<Vehicle> Vehicles;

struct LanePredictions {
  Vehicles vehicles;
  double speed;
  bool has_vehicle_ahead;
  bool has_vehicle_behind;
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;
};

struct PredictionData {
  std::vector<LanePredictions> lanes;
};

struct EgoStatus {
  std::uint8_t lane_id{1U};  // 0=left, 1=middle, 2=right
  float s{0.0F};
  float speed{0.0F};
};

}  // namespace udacity

#endif  // DATA_TYPES_H

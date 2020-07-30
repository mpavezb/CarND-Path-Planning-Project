#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <iostream>
#include <string>
#include <vector>

namespace udacity {

typedef std::vector<double> Path;
typedef std::vector<Path> Paths;

struct Parameters {
  // map
  std::string map_filename{"../data/highway_map.csv"};
  float max_s = 6945.554;
  float lane_width{4.0F};
  float speed_limit{50.0 / 2.237F};

  // lifecycle
  float time_step_{0.02F};

  // target
  std::uint8_t goal_lane_id{1};  // 0=left, 1=middle, 2=right
  float goal_s{max_s};

  // control
  float desired_speed{49.5F / 2.237F};
  float min_distance_to_front_object{30};

  // trajectory generation
  int path_size_{50};
  int n_anchors_{5};
  float look_ahead_distance_{50.0F};
  float acceleration{0.1};
  float deceleration{0.2};
};

struct Map {
  double max_s{0.0};
  Path waypoints_x;
  Path waypoints_y;
  Path waypoints_s;
  Path waypoints_dx;
  Path waypoints_dy;
};

struct Point {
  double x;
  double y;
};

struct AnchorReference {
  double s;
  double d;
  double x1;
  double x2;
  double y1;
  double y2;
  double yaw;
};

struct SplineAnchors {
  Path x;
  Path y;
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

struct Vehicle {
  int id;
  double s;
  double d;
  double speed;
  double lane_id;
};

typedef std::vector<Vehicle> Vehicles;
typedef std::vector<FusedObject> FusedObjects;

struct TelemetryPacket {
  // Main car's localization Data
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;  // mps!

  // A list of all other cars on the same side the road.
  FusedObjects sensor_fusion;

  // Unvisited points from previous trajectory
  Path last_trajectory_x;
  Path last_trajectory_y;

  // Previous path's end s and d values
  double end_path_s;
  double end_path_d;
};

struct PredictionData {
  Vehicles vehicles;
  std::vector<double> lane_speeds;  // Refactor
};

struct EgoStatus {
  std::uint8_t lane_id{1};  // 0=left, 1=middle, 2=right
  float s{0.0F};
  float speed{0.0F};
};

}  // namespace udacity

#endif  // DATA_TYPES_H

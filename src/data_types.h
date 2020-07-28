#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <iostream>
#include <string>
#include <vector>

namespace udacity {

typedef std::vector<double> Path;
typedef std::vector<Path> Paths;

struct Point {
  double x;
  double y;
};

struct Pose {
  double x;
  double y;
  double yaw;
  double s;
  double d;
};

struct SplineAnchors {
  Path x;
  Path y;
  Pose reference;
};

enum class TrajectoryAction : std::uint8_t {
  kKeepLane,
  kChangeLaneLeft,
  kChangeLaneRight,
  kPrepareChangeLaneLeft,
  kPrepareChangeLaneRight
};

struct TrajectoryCharacteristics {
  TrajectoryAction action;
  double speed;
  double cost;
  std::uint8_t intended_lane_id;  // expected lane id after successful maneuver.
  std::uint8_t endpoint_lane_id;  // Lane id for this trajectory's endpoint.
};

struct Trajectory {
  Path x;
  Path y;
  TrajectoryCharacteristics characteristics;

  bool operator<(const Trajectory& other) const {
    return this->characteristics.cost < other.characteristics.cost;
  }
};

struct Map {
  double max_s{0.0};
  std::string filename;
  Path waypoints_x;
  Path waypoints_y;
  Path waypoints_s;
  Path waypoints_dx;
  Path waypoints_dy;
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

typedef std::vector<FusedObject> SensorFusionList;

struct TelemetryPacket {
  // Main car's localization Data
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;  // mps!

  // A list of all other cars on the same side the road.
  SensorFusionList sensor_fusion;

  // Unvisited points from previous trajectory
  Trajectory last_trajectory;

  // Previous path's end s and d values
  double end_path_s;
  double end_path_d;
};

struct PredictionData {
  SensorFusionList sensor_fusion;
  std::vector<double> lane_speeds;  // Refactor
};

struct EnvironmentData {
  float lane_width{4.0F};
  float speed_limit{50.0 / 2.237F};
};

struct TargetData {
  std::uint8_t lane_id{1};  // 0=left, 1=middle, 2=right
  float speed{49.5F / 2.237F};
};

struct EgoStatus {
  std::uint8_t lane_id{1};  // 0=left, 1=middle, 2=right
  float speed{0.0F};
};

}  // namespace udacity

#endif  // DATA_TYPES_H

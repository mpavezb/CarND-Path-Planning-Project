#ifndef DATA_TYPES_H
#define DATA_TYPES_H

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

struct Trajectory {
  Path x;
  Path y;
  TrajectoryAction action;
  double cost;

  bool operator<(const Trajectory& other) const {
    return this->cost < other.cost;
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
  double car_speed;

  // A list of all other cars on the same side the road.
  SensorFusionList sensor_fusion;

  // Unvisited points from previous trajectory
  Trajectory last_trajectory;

  // Previous path's end s and d values
  double end_path_s;
  double end_path_d;
};

}  // namespace udacity

#endif  // DATA_TYPES_H

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

struct Frenet {
  double s;
  double d;
  Frenet() : s(.0), d(.0) {}
  Frenet(double s, double d) : s(s), d(d) {}
};

typedef std::vector<Point> Path;
typedef std::vector<Frenet> FrenetPath;

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
  FrenetPath frenet_path;
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

struct AnchorReference {
  double s;
  double d;
  double x1;
  double x2;
  double y1;
  double y2;
  double yaw;
};

typedef Path SplineAnchors;

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
  bool is_valid{false};
};

struct Trajectory {
  Path path;
  FrenetPath frenet_path;
  TrajectoryCharacteristics characteristics;

  bool operator<(const Trajectory& other) const {
    return this->characteristics.cost < other.characteristics.cost;
  }
};

std::ostream& operator<<(std::ostream& os, const Point& p) {
  return os << "(" << p.x << "," << p.y << ")";
}

std::ostream& operator<<(std::ostream& os, const AnchorReference& r) {
  return os << "(s,d): (" << r.s << "," << r.d << "), p1: " << Point(r.x1, r.y1)
            << ", p2: " << Point(r.x2, r.y2) << ", yaw: " << r.yaw;
}

std::ostream& operator<<(std::ostream& os, const TrajectoryAction& action) {
  switch (action) {
    case TrajectoryAction::kKeepLane:
      os << "kKeepLane";
      break;
    case TrajectoryAction::kChangeLaneLeft:
      os << "kChangeLaneLeft";
      break;
    case TrajectoryAction::kChangeLaneRight:
      os << "kChangeLaneRight";
      break;
    case TrajectoryAction::kPrepareChangeLaneLeft:
      os << "kPrepareChangeLaneLeft";
      break;
    case TrajectoryAction::kPrepareChangeLaneRight:
      os << "kPrepareChangeLaneRight";
      break;
  }
  return os;
}

}  // namespace udacity

#endif  // DATA_TYPES_H

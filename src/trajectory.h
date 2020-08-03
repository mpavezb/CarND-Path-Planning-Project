#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include "data_types.h"

namespace udacity {

struct AnchorReference {
  double s;
  double d;
  double x1;
  double x2;
  double y1;
  double y2;
  double yaw;
};

std::ostream& operator<<(std::ostream& os, const AnchorReference& r) {
  return os << "(s,d): (" << r.s << "," << r.d << "), p1: " << Point(r.x1, r.y1)
            << ", p2: " << Point(r.x2, r.y2) << ", yaw: " << r.yaw;
}

typedef Path SplineAnchors;

enum class TrajectoryAction : std::uint8_t {
  kKeepLane,
  kChangeLaneLeft,
  kChangeLaneRight,
  kPrepareChangeLaneLeft,
  kPrepareChangeLaneRight
};

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
  TrajectoryCharacteristics characteristics;

  bool operator<(const Trajectory& other) const {
    return this->characteristics.cost < other.characteristics.cost;
  }
};

}  // namespace udacity

#endif  // TRAJECTORY_H_

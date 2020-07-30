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
  bool is_valid{true};
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

#ifndef TRAJECTORY_VALIDATOR_H_
#define TRAJECTORY_VALIDATOR_H_

#include "data_types.h"

namespace udacity {

class TrajectoryValidator {
 public:
  bool isTrajectoryValid(const Trajectory &trajectory) {
    if (trajectory.action == TrajectoryAction::kKeepLane) return true;
    return false;
  }
  double getTrajectoryCost(const Trajectory &trajectory) {
    if (trajectory.action == TrajectoryAction::kKeepLane) return 0.0;
    return 1.0;
  }
};

}  // namespace udacity

#endif  // TRAJECTORY_VALIDATOR_H_

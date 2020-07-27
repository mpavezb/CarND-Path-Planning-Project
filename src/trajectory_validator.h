#ifndef TRAJECTORY_VALIDATOR_H_
#define TRAJECTORY_VALIDATOR_H_

#include "data_types.h"

namespace udacity {

class TrajectoryValidator {
 public:
  double goal_distance_cost(int goal_lane, int intended_lane, int final_lane,
                            double distance_to_goal) {
    // The cost increases with both the distance of intended lane from the goal
    //   and the distance of the final lane from the goal. The cost of being out
    //   of the goal lane also becomes larger as the vehicle approaches the
    //   goal.
    int delta_d = 2.0 * goal_lane - intended_lane - final_lane;
    double cost = 1 - exp(-(std::abs(delta_d) / distance_to_goal));

    return cost;
  }

  bool isTrajectoryValid(const Trajectory &trajectory) { return true; }

  double getTrajectoryCost(const Trajectory &trajectory) {
    if (trajectory.action == TrajectoryAction::kPrepareChangeLaneRight)
      return 0.0;
    if (trajectory.action == TrajectoryAction::kKeepLane) return 0.5;
    return 1.0;
  }
};

}  // namespace udacity

#endif  // TRAJECTORY_VALIDATOR_H_

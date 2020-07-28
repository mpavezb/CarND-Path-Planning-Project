#ifndef TRAJECTORY_VALIDATOR_H_
#define TRAJECTORY_VALIDATOR_H_

#include <cmath>
#include <memory>

#include "data_types.h"

namespace udacity {

class CostFunction {
 public:
  /**
   * Returns cost in range [0,1]. Good trajectories yield lower cost.
   */
  virtual double getCost(const Trajectory &, const PredictionData &) = 0;
};

class SpeedCostFunction : public CostFunction {
 public:
  double getCost(const Trajectory &trajectory,
                 const PredictionData &predictions) override {
    return 0;
    // ARGS
    double current_speed = 0;
    // PARAMS
    double speed_buffer = 2;
    double stop_cost = 0.8;
    double speed_limit = 22.0;  // TODO
    double weight = 1.0;

    double cost = 0;
    double best_speed = speed_limit - speed_buffer;
    if (current_speed < best_speed) {
      cost = stop_cost * (best_speed - current_speed) / best_speed;
    } else if (best_speed < current_speed && current_speed < speed_limit) {
      cost = (current_speed - best_speed) / speed_buffer;
    } else {
      cost = 1;
    }
    return cost * weight;
  }
};

class GoalDistanceCostFunction : public CostFunction {
 public:
  double getCost(const Trajectory &trajectory,
                 const PredictionData &predictions) override {
    return 0;
    // ARGS:
    int goal_lane = 0;
    int intended_lane = 0;
    int final_lane = 0;
    double distance_to_goal = 0;
    // PARAMS: ?
    double weight = 1.0;

    // The cost increases with both the distance of intended lane from the
    // goal
    //   and the distance of the final lane from the goal. The cost of being
    //   out of the goal lane also becomes larger as the vehicle approaches
    //   the goal.
    int delta_d = 2.0 * goal_lane - intended_lane - final_lane;
    double cost = 1 - exp(-(std::abs(delta_d) / distance_to_goal));

    return cost * weight;
  }
};

class InefficientLaneCostFunction : public CostFunction {
 public:
  double getCost(const Trajectory &trajectory,
                 const PredictionData &predictions) override {
    return 0;
    // ARGS
    int target_speed = 0;
    int intended_lane = 0;
    int final_lane = 0;
    double weight = 1.0;
    const std::vector<int> lane_speeds;

    // Cost becomes higher for trajectories with intended lane and final
    // lane
    //   that have traffic slower than target_speed.
    double speed_intended = lane_speeds[intended_lane];
    double speed_final = lane_speeds[final_lane];
    double cost =
        (2.0 * target_speed - speed_intended - speed_final) / target_speed;

    return cost * weight;
  }
};

class TrajectoryValidator {
 public:
  TrajectoryValidator() {
    cost_functions.emplace_back(new SpeedCostFunction());
    cost_functions.emplace_back(new GoalDistanceCostFunction());
    cost_functions.emplace_back(new InefficientLaneCostFunction());
  }

  bool isTrajectoryValid(const Trajectory &trajectory) { return true; }

  double getTrajectoryCost(const Trajectory &trajectory,
                           const PredictionData &predictions) {
    double cost = 0;
    for (auto &cost_function : cost_functions) {
      cost += cost_function->getCost(trajectory, predictions);
    }
    return cost;
  }

 private:
  std::vector<std::unique_ptr<CostFunction>> cost_functions;
};

}  // namespace udacity

#endif  // TRAJECTORY_VALIDATOR_H_

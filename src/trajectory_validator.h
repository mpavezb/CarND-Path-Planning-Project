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

/**
 * Implements a 3-piecewise linear cost function based on the intended speed,
 * the target speed a
 */
class SpeedCostFunction : public CostFunction {
 public:
  double getCost(const Trajectory &trajectory,
                 const PredictionData &) override {
    double cost = 0;
    double speed = trajectory.characteristics.speed;
    if (speed < kBestSpeed) {
      cost = kStopCost * (kBestSpeed - speed) / kBestSpeed;
    } else if (kBestSpeed < speed && speed < kSpeedLimit) {
      cost = (speed - kBestSpeed) / kSpeedBuffer;
    } else {
      cost = 1;
    }
    return cost * kFunctionWeight;
  }

 private:
  double kStopCost = 0.8;
  double kSpeedBuffer{2.0 / 2.237};
  double kSpeedLimit{50.0 / 2.237};
  double kBestSpeed{kSpeedLimit - kSpeedBuffer};
  double kFunctionWeight{1.0};
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

    // The cost increases with both the distance of intended lane from the
    // goal
    //   and the distance of the final lane from the goal. The cost of being
    //   out of the goal lane also becomes larger as the vehicle approaches
    //   the goal.
    int delta_d = 2.0 * goal_lane - intended_lane - final_lane;
    double cost = 1 - exp(-(std::abs(delta_d) / distance_to_goal));

    return cost * kFunctionWeight;
  }

 private:
  double kFunctionWeight{1.0};
};

/**
 * Cost becomes higher for trajectories with intended lane and endpoint
 * lane that have traffic slower than the trajectory speed.
 */
class InefficientLaneCostFunction : public CostFunction {
 public:
  double getCost(const Trajectory &trajectory,
                 const PredictionData &predictions) override {
    const auto lane_speeds = predictions.lane_speeds;

    int target_speed = trajectory.characteristics.speed;
    int intended_lane = trajectory.characteristics.intended_lane_id;
    int endpoint_lane = trajectory.characteristics.endpoint_lane_id;

    double intended_lane_speed = lane_speeds[intended_lane];
    double endpoint_lane_speed = lane_speeds[endpoint_lane];
    double cost =
        (2.0 * target_speed - intended_lane_speed - endpoint_lane_speed) /
        target_speed;

    return cost * kFunctionWeight;
  }

 private:
  double kFunctionWeight{1.0};
};

class TrajectoryValidator {
 public:
  TrajectoryValidator() {
    cost_functions.emplace_back(new SpeedCostFunction());
    cost_functions.emplace_back(new GoalDistanceCostFunction());
    cost_functions.emplace_back(new InefficientLaneCostFunction());
  }

  bool isActionLaneInRoad(const Trajectory &trajectory) {
    if (ego_.lane_id == 0) {
      if (trajectory.characteristics.action ==
              TrajectoryAction::kPrepareChangeLaneLeft ||
          trajectory.characteristics.action ==
              TrajectoryAction::kChangeLaneLeft)
        return false;
    }
    if (ego_.lane_id == 2) {
      if (trajectory.characteristics.action ==
              TrajectoryAction::kPrepareChangeLaneRight ||
          trajectory.characteristics.action ==
              TrajectoryAction::kChangeLaneRight)
        return false;
    }
    return true;
  }

  bool isTrajectoryValid(const Trajectory &trajectory) {
    bool is_valid = true;
    is_valid = is_valid and isActionLaneInRoad(trajectory);
    return is_valid;
  }

  double getTrajectoryCost(const Trajectory &trajectory,
                           const PredictionData &predictions) {
    double cost = 0;
    for (auto &cost_function : cost_functions) {
      cost += cost_function->getCost(trajectory, predictions);
    }
    return cost;
  }

  void setEgoStatus(const EgoStatus &ego) { ego_ = ego; }
  void setPredictionData(const PredictionData &predictions) {
    predictions_ = predictions;
  }

 private:
  EgoStatus ego_;
  PredictionData predictions_;
  std::vector<std::unique_ptr<CostFunction>> cost_functions;
};

}  // namespace udacity

#endif  // TRAJECTORY_VALIDATOR_H_

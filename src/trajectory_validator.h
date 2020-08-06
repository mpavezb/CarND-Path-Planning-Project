#ifndef TRAJECTORY_VALIDATOR_H_
#define TRAJECTORY_VALIDATOR_H_

#include <cmath>
#include <memory>

#include "data_types.h"
#include "helpers.h"
#include "parameters.h"

namespace udacity {

class CostFunction {
 public:
  /**
   * Returns cost in range [0,1]. Good trajectories yield lower cost.
   */
  virtual double getCost(const Trajectory &, const PredictionData &,
                         const EgoStatus &, const Parameters &) = 0;
};

/**
 * Implements a 3-piecewise linear cost function based on the trajector
 * final speed and the optimal speed.
 */
class SpeedCostFunction : public CostFunction {
 public:
  double getCost(const Trajectory &trajectory, const PredictionData &,
                 const EgoStatus &, const Parameters &parameters) override {
    double kSpeedLimit = parameters.speed_limit;
    double kBestSpeed = parameters.desired_speed;
    double kSpeedBuffer = {kSpeedLimit - kBestSpeed};

    double cost = 0;
    double speed = trajectory.characteristics.speed;
    if (speed < kBestSpeed) {
      cost = kStopCost * (1.0 - speed / kBestSpeed);
    } else if (kBestSpeed < speed && speed < kSpeedLimit) {
      cost = (speed - kBestSpeed) / kSpeedBuffer;
    } else {
      cost = 1;
    }

    // std::cout << "[Cost:Speed]" << trajectory.characteristics.action
    //           << ", speed: " << speed << ", best: " << kBestSpeed
    //           << ", c: " << cost * kFunctionWeight << std::endl;

    return cost * kFunctionWeight;
  }

 private:
  double kStopCost = 0.9;
  double kFunctionWeight{10};
};

/**
 *  The cost increases with both the distance of intended lane from the
 *  goal and the distance of the endpoints lane from the goal.
 *  The cost of being out of the goal lane also becomes larger as the
 *  vehicle approaches the goal.
 */
class GoalDistanceCostFunction : public CostFunction {
 public:
  double getCost(const Trajectory &trajectory,
                 const PredictionData &predictions, const EgoStatus &ego,
                 const Parameters &parameters) override {
    int goal_lane = parameters.goal_lane_id;
    int intended_lane = trajectory.characteristics.intended_lane_id;
    int endpoint_lane = trajectory.characteristics.endpoint_lane_id;
    double distance_to_goal = parameters.goal_s - ego.s;

    int delta_d = 2.0 * goal_lane - intended_lane - endpoint_lane;
    double cost = 1 - exp(-(std::abs(delta_d) / distance_to_goal));

    // std::cout << "[Cost:DistanceToGoal]" << trajectory.characteristics.action
    //           << ", goal id: " << goal_lane
    //           << ", intended id: " << intended_lane
    //           << ", endpoint id: " << endpoint_lane
    //           << ", c: " << cost * kFunctionWeight << std::endl;

    return cost * kFunctionWeight;
  }

 private:
  double kFunctionWeight{100};
};

/**
 * Cost becomes higher for trajectories with intended lane and endpoint
 * lane that have traffic slower than the trajectory speed.
 */
class InefficientLaneCostFunction : public CostFunction {
 public:
  double getCost(const Trajectory &trajectory,
                 const PredictionData &predictions, const EgoStatus &ego,
                 const Parameters &parameters) override {
    int current_speed = parameters.desired_speed;
    int intended_lane = trajectory.characteristics.intended_lane_id;
    int endpoint_lane = trajectory.characteristics.endpoint_lane_id;

    double intended_lane_speed = predictions.lanes[intended_lane].speed;
    double endpoint_lane_speed = predictions.lanes[endpoint_lane].speed;
    double cost =
        2.0 - (intended_lane_speed + endpoint_lane_speed) / current_speed;
    cost = fmax(0.0, cost);

    // std::cout << "[Cost:InefficientLane]" <<
    // trajectory.characteristics.action
    //           << ", ispeed: " << intended_lane_speed
    //           << ", espeed: " << endpoint_lane_speed
    //           << ", cspeed: " << current_speed
    //           << ", c: " << cost * kFunctionWeight << std::endl;

    return cost * kFunctionWeight;
  }

 private:
  double kFunctionWeight{15};
};

class PreferEmptyLaneCostFunction : public CostFunction {
 public:
  double getCost(const Trajectory &trajectory,
                 const PredictionData &predictions, const EgoStatus &,
                 const Parameters &parameters) override {
    int intended_lane_id = trajectory.characteristics.intended_lane_id;
    auto &lane = predictions.lanes[intended_lane_id];

    double d_max = parameters.cost_empty_lane_dmax;
    double c_max = parameters.cost_empty_lane_cmax;

    double cost = 0;
    if (lane.has_vehicle_ahead and lane.vehicle_ahead.distance < d_max) {
      double distance = lane.vehicle_ahead.distance;
      cost = c_max * (1.0 - distance / d_max);
    }
    // std::cout << "[Cost:EmptyLane]" << trajectory.characteristics.action
    //           << ", ilane: " << intended_lane_id
    //           << ", c: " << cost * kFunctionWeight << std::endl;

    return cost * kFunctionWeight;
  }

 private:
  double kFunctionWeight{10};
};

class TrajectoryValidator {
 public:
  TrajectoryValidator(const Parameters &parameters) : parameters_(parameters) {
    cost_functions.emplace_back(new SpeedCostFunction());
    cost_functions.emplace_back(new GoalDistanceCostFunction());
    cost_functions.emplace_back(new InefficientLaneCostFunction());
    cost_functions.emplace_back(new PreferEmptyLaneCostFunction());
  }

  bool isCollisionPredictedInLaneChange(const Trajectory &trajectory) {
    const auto action = trajectory.characteristics.action;
    const auto intended_lane_id = trajectory.characteristics.intended_lane_id;
    const auto &intended_lane = predictions_.lanes[intended_lane_id];

    Frenet threshold{parameters_.collision_th_s, parameters_.collision_th_d};
    if (action == TrajectoryAction::kChangeLaneLeft ||
        action == TrajectoryAction::kChangeLaneRight) {
      for (const auto &vehicle : intended_lane.vehicles) {
        if (arePathsColliding(trajectory.frenet_path, vehicle.frenet_path,
                              threshold, parameters_.collision_steps)) {
          // std::cout << action << " is invalid due to collision" << std::endl;
          return true;
        }
      }
    }
    return false;
  }

  bool isActionLaneValid(const Trajectory &trajectory) {
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
    bool is_valid = trajectory.characteristics.is_valid;
    is_valid = is_valid and isActionLaneValid(trajectory);
    is_valid = is_valid and not isCollisionPredictedInLaneChange(trajectory);
    return is_valid;
  }

  double getTrajectoryCost(const Trajectory &trajectory) {
    double cost = 0;
    for (const auto &cost_function : cost_functions) {
      cost +=
          cost_function->getCost(trajectory, predictions_, ego_, parameters_);
    }
    return cost;
  }

  void setEgoStatus(const EgoStatus &ego) { ego_ = ego; }
  void setPredictionData(const PredictionData &predictions) {
    predictions_ = predictions;
  }

 private:
  Parameters parameters_;
  EgoStatus ego_;
  PredictionData predictions_;
  std::vector<std::unique_ptr<CostFunction>> cost_functions;
};

}  // namespace udacity

#endif  // TRAJECTORY_VALIDATOR_H_

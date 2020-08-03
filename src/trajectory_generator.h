#ifndef TRAJECTORY_GENERATOR_H_
#define TRAJECTORY_GENERATOR_H_

#include <memory>
#include <vector>

#include "data_types.h"
#include "helpers.h"
#include "third_party/spline.h"
#include "trajectory.h"

namespace udacity {

/**
 * Provides trajectories for different scenarios, based on the optimal speed for
 * it.
 */
class TrajectoryGenerator {
 public:
  TrajectoryGenerator(std::shared_ptr<Map> map, const Parameters& parameters)
      : map_(map), parameters_(parameters) {}

  void setTelemetry(const TelemetryPacket& telemetry) {
    telemetry_ = telemetry;
  }
  void setEgoStatus(const EgoStatus& ego) { ego_ = ego; }

  /**
   * Anchors are based on the previous path's endpoint whenever possible.
   * Otherwise, the car is used as starting reference.
   *
   * Note that previous path end point is always further away than the car.
   */
  void updateAnchorReference() {
    AnchorReference& ref = anchor_reference_;
    auto last_path_size = telemetry_.last_path.size();
    if (last_path_size < 2) {
      // based only on car
      ref.yaw = telemetry_.car_yaw;
      ref.x2 = telemetry_.car_x;
      ref.y2 = telemetry_.car_y;
      ref.s = telemetry_.car_s;
      ref.d = telemetry_.car_d;
      ref.x1 = ref.x2 - cos(ref.yaw);
      ref.y1 = ref.y2 - sin(ref.yaw);
      // std::cout << "Car Based AnchorReference: " << ref << std::endl;
    } else {
      // based only on previous endpoints
      // TODO: This is problematic if last path was generated with low speed
      ref.x1 = telemetry_.last_path[last_path_size - 2].x;
      ref.y1 = telemetry_.last_path[last_path_size - 2].y;
      ref.x2 = telemetry_.last_path[last_path_size - 1].x;
      ref.y2 = telemetry_.last_path[last_path_size - 1].y;
      ref.yaw = atan2(ref.y2 - ref.y1, ref.x2 - ref.x1);
      ref.s = telemetry_.end_path_s;
      ref.d = telemetry_.car_d;
      // std::cout << "Last Path Based AnchorReference: " << ref << std::endl;
    }
  }

  void step() { updateAnchorReference(); }

  Trajectory getTrajectoryForAction(TrajectoryAction action,
                                    const PredictionData& predictions) {
    Trajectory result;
    switch (action) {
      case TrajectoryAction::kKeepLane:
        result = generateKeepLaneTrajectory(predictions);
        break;
      case TrajectoryAction::kPrepareChangeLaneLeft:
        result = generatePrepareLaneChangeLeftTrajectory(predictions);
        break;
      case TrajectoryAction::kChangeLaneLeft:
        result = generateLaneChangeLeftTrajectory(predictions);
        break;
      case TrajectoryAction::kPrepareChangeLaneRight:
        result = generatePrepareLaneChangeRightTrajectory(predictions);
        break;
      case TrajectoryAction::kChangeLaneRight:
        result = generateLaneChangeRightTrajectory(predictions);
        break;
    }
    result.characteristics.action = action;
    return result;
  }

  /**
   * Generate spline anchors for interpolation.
   */
  SplineAnchors generateSplineAnchors(std::uint8_t intended_lane_id) {
    SplineAnchors anchors;
    const AnchorReference& ref = anchor_reference_;
    anchors.push_back({ref.x1, ref.y1});
    anchors.push_back({ref.x2, ref.y2});

    // Add extra anchors
    int n_missing_anchors = parameters_.n_anchors_ - 2;
    for (int i = 0; i < n_missing_anchors; ++i) {
      float spacing = parameters_.look_ahead_distance_ / n_missing_anchors;
      float s = ref.s + (i + 1) * spacing;
      float d = parameters_.lane_width * (0.5 + intended_lane_id);
      std::vector<double> anchor =
          getXY(s, d, map_->waypoints_s, map_->waypoints_x, map_->waypoints_y);
      anchors.push_back({anchor[0], anchor[1]});
    }
    return anchors;
  }

  /**
   * Transform anchors to ego frame to ensure they are as horizontal as
   * possible, so that for any given x, there is only one y value.
   */
  SplineAnchors transformToEgo(const SplineAnchors& anchors) {
    SplineAnchors result;
    const AnchorReference& ref = anchor_reference_;
    result.resize(parameters_.n_anchors_);

    for (int i = 0; i < parameters_.n_anchors_; i++) {
      double shift_x = anchors[i].x - ref.x2;
      double shift_y = anchors[i].y - ref.y2;
      result[i].x = shift_x * cos(0 - ref.yaw) - shift_y * sin(0 - ref.yaw);
      result[i].y = shift_x * sin(0 - ref.yaw) + shift_y * cos(0 - ref.yaw);
    }
    return result;
  }

  Point transformToMap(const Point& p) {
    Point result;
    const AnchorReference& ref = anchor_reference_;
    result.x = p.x * cos(ref.yaw) - p.y * sin(ref.yaw);
    result.y = p.x * sin(ref.yaw) + p.y * cos(ref.yaw);
    result.x += ref.x2;
    result.y += ref.y2;
    return result;
  }

  bool areAnchorsValid(const SplineAnchors& ego_anchors) {
    for (int i = 1; i < ego_anchors.size(); ++i) {
      const auto p1 = ego_anchors[i - 1];
      const auto p2 = ego_anchors[i];
      if (p1.x >= p2.x) {
        std::cerr << "Got invalid anchors:" << std::endl;
        printPath(ego_anchors, "anchors");
        return false;
      }
    }
    return true;
  }

  /**
   * Generate interpolation points between anchors.
   * Points are evenly spaced in x axis, so that desired speed is kept.
   */
  Trajectory interpolateMissingPoints(const SplineAnchors& ego_anchors,
                                      double speed) {
    Trajectory result;
    result.path = telemetry_.last_path;
    int missing_points = parameters_.path_size_ - result.path.size();

    if (not areAnchorsValid(ego_anchors)) {
      return generateInvalidTrajectory();
    }

    tk::spline gen;
    gen.set_points(getPathX(ego_anchors), getPathY(ego_anchors));

    double target_x = parameters_.look_ahead_distance_;
    double target_y = gen(target_x);
    double target_dist = distance(0, 0, target_x, target_y);
    double N = target_dist / (speed * parameters_.time_step_);

    for (int i = 0; i < missing_points; ++i) {
      Point ego_point;
      ego_point.x = (i + 1) * target_x / N;
      ego_point.y = gen(ego_point.x);

      Point map_point = transformToMap(ego_point);
      result.path.push_back({map_point.x, map_point.y});
    }
    result.characteristics.speed = speed;
    result.characteristics.is_valid = true;

    return result;
  }

  double getOptimalSpeed(const PredictionData& predictions,
                         std::uint8_t intended_lane_id,
                         std::uint8_t endpoint_lane_id) {
    const AnchorReference& ref = anchor_reference_;
    const auto& intended_lane = predictions.lanes[intended_lane_id];
    const auto& endpoint_lane = predictions.lanes[endpoint_lane_id];

    // TODO: Need to know when to start stopping to avoid colliding front
    // vehicle!
    double delta_speed = 0;
    bool has_near_vehicle_ahead =
        endpoint_lane.has_vehicle_ahead and endpoint_lane.vehicle_ahead.is_near;
    bool has_near_vehicle_behind = endpoint_lane.has_vehicle_behind and
                                   endpoint_lane.vehicle_behind.is_near;
    if (has_near_vehicle_ahead) {
      double target_speed;
      if (has_near_vehicle_behind) {
        // keep speed of slower vehicle
        double vehicle_ahead_speed = endpoint_lane.vehicle_ahead.speed;
        double vehicle_behind_speed = endpoint_lane.vehicle_behind.speed;
        target_speed = fmin(vehicle_ahead_speed, vehicle_behind_speed);
      } else {
        // slow down as much as needed to match speeds
        target_speed = endpoint_lane.vehicle_ahead.speed;
      }
      double speed_diff = fabs(ego_.speed - target_speed);
      delta_speed = -1.0 * fmin(speed_diff, parameters_.deceleration);

    } else {
      // TODO: Dont increase too much if there is a vehicle ahead. Keep speed
      // instead. This will result in a sudden speed increase.
      // Keep max velocity possible
      delta_speed = parameters_.acceleration;
    }

    // TODO: this helps avoiding invalid anchor generation
    // if speed is near zero, then last trajectory points
    // will be too near for anchors to be valid.
    double minimum_speed = 0.1;

    return fmax(minimum_speed,
                fmin(ego_.speed + delta_speed, parameters_.desired_speed));

    // PrepareFor...
    // double speed_actual = getSpeedForecast(predictions, endpoint_lane_id);
    // double speed_next = getSpeedForecast(predictions, intended_lane_id);

    // double speed;
    // bool is_object_behind = false;
    // if (is_object_behind) {
    //   // keep speed of current lane so as to not collide with car behind
    //   speed = speed_actual;
    // } else {
    //   // prefer the lowest speed from both lanes
    //   speed = fmin(speed_actual, speed_next);
    // }
  }

  bool isLaneChangePossible(const PredictionData& predictions,
                            std::uint8_t intended_lane_id) {
    const auto& lane = predictions.lanes[intended_lane_id];
    for (const auto& vehicle : lane.vehicles) {
      if (vehicle.is_ahead and vehicle.predicted_distance <
                                   parameters_.lane_change_gap_ahead_length) {
        return false;
      }
      if (vehicle.is_behind and vehicle.predicted_distance <
                                    parameters_.lane_change_gap_behind_length) {
        return false;
      }
    }
    return true;
  }

  /**
   * Spline based Trajectory Generation.
   *
   * Anchor points are generated based on the current state and the goal
   * state. Continuity is ensured by using the previously generated path as
   * a baseline and a spline as interpolator for next path points.
   *
   * Paths are expected to have a fixed length. The generator appends
   * missing points to the previously generated trajectory.
   */
  Trajectory generateTrajectoryFromSpline(std::uint8_t intended_lane_id,
                                          std::uint8_t endpoint_lane_id,
                                          double speed) {
    // Generate spline
    SplineAnchors anchors = generateSplineAnchors(endpoint_lane_id);
    SplineAnchors ego_anchors = transformToEgo(anchors);
    Trajectory trajectory = interpolateMissingPoints(ego_anchors, speed);
    trajectory.characteristics.endpoint_lane_id = endpoint_lane_id;
    trajectory.characteristics.intended_lane_id = intended_lane_id;
    return trajectory;
  }

  Trajectory generateInvalidTrajectory() {
    Trajectory invalid_trajectory;
    invalid_trajectory.characteristics.is_valid = false;
    return invalid_trajectory;
  }

  Trajectory generateTrajectory(const PredictionData& predictions,
                                std::uint8_t intended_lane_id,
                                std::uint8_t endpoint_lane_id) {
    double speed =
        getOptimalSpeed(predictions, intended_lane_id, endpoint_lane_id);
    return generateTrajectoryFromSpline(intended_lane_id, endpoint_lane_id,
                                        speed);
  }

  Trajectory generateKeepLaneTrajectory(const PredictionData& predictions) {
    std::uint8_t intended_lane_id = ego_.lane_id;
    std::uint8_t endpoint_lane_id = ego_.lane_id;
    return generateTrajectory(predictions, intended_lane_id, endpoint_lane_id);
  }

  Trajectory generatePrepareLaneChangeLeftTrajectory(
      const PredictionData& predictions) {
    std::uint8_t intended_lane_id = fmax(0, ego_.lane_id - 1);
    std::uint8_t endpoint_lane_id = ego_.lane_id;
    return generateTrajectory(predictions, intended_lane_id, endpoint_lane_id);
  }

  Trajectory generatePrepareLaneChangeRightTrajectory(
      const PredictionData& predictions) {
    std::uint8_t intended_lane_id = fmin(2, ego_.lane_id + 1);
    std::uint8_t endpoint_lane_id = ego_.lane_id;
    return generateTrajectory(predictions, intended_lane_id, endpoint_lane_id);
  }

  Trajectory generateLaneChangeTrajectory(const PredictionData& predictions,
                                          std::uint8_t intended_lane_id,
                                          std::uint8_t endpoint_lane_id) {
    if (!isLaneChangePossible(predictions, intended_lane_id)) {
      return generateInvalidTrajectory();
    }
    return generateTrajectory(predictions, intended_lane_id, endpoint_lane_id);
  }

  Trajectory generateLaneChangeLeftTrajectory(
      const PredictionData& predictions) {
    std::uint8_t intended_lane_id = fmax(0, ego_.lane_id - 1);
    std::uint8_t endpoint_lane_id = intended_lane_id;
    return generateLaneChangeTrajectory(predictions, intended_lane_id,
                                        endpoint_lane_id);
  }

  Trajectory generateLaneChangeRightTrajectory(
      const PredictionData& predictions) {
    std::uint8_t intended_lane_id = fmin(2, ego_.lane_id + 1);
    std::uint8_t endpoint_lane_id = intended_lane_id;
    return generateLaneChangeTrajectory(predictions, intended_lane_id,
                                        endpoint_lane_id);
  }

 private:
  // Input Data
  std::shared_ptr<Map> map_;
  Parameters parameters_;
  TelemetryPacket telemetry_;
  EgoStatus ego_;
  AnchorReference anchor_reference_;
};

}  // namespace udacity

#endif  // TRAJECTORY_GENERATOR_H_

#ifndef TRAJECTORY_GENERATOR_H_
#define TRAJECTORY_GENERATOR_H_

#include <iostream>
#include <memory>
#include <vector>

#include "data_types.h"
#include "helpers.h"
#include "third_party/spline.h"

namespace udacity {

class TrajectoryGenerator {
 public:
  void setMap(std::shared_ptr<Map> map) { map_ = map; }
  void setTelemetry(const TelemetryPacket& telemetry) {
    telemetry_ = telemetry;
    updateCurrentLaneId(telemetry.car_d);
  }
  void updateCurrentLaneId(double car_d) {
    current_lane_id_ = fmax(fmin(2, floor(car_d / lane_width_)), 0);
  }

  Trajectory getTrajectoryForAction(TrajectoryAction action,
                                    const PredictionData& predictions) {
    Trajectory result;
    switch (action) {
      case TrajectoryAction::kKeepLane:
        result = generateKeepLaneTrajectory(telemetry_, predictions);
        break;
      case TrajectoryAction::kPrepareChangeLaneLeft:
      case TrajectoryAction::kChangeLaneLeft:
        result = generateLaneChangeLeftTrajectory(telemetry_, predictions);
        break;
      case TrajectoryAction::kPrepareChangeLaneRight:
      case TrajectoryAction::kChangeLaneRight:
        result = generateLaneChangeRightTrajectory(telemetry_, predictions);
        break;
    }
    result.action = action;
    return result;
  }

  /**
   * Generate spline anchors for interpolation.
   *
   * Anchors are based on the previous path's endpoint whenever possible.
   * Otherwise, the car is used as starting reference.
   *
   * Note that previous path end point is always further away than the car.
   */
  SplineAnchors generateSplineAnchors(const TelemetryPacket& telemetry,
                                      std::uint8_t target_lane_id) {
    SplineAnchors anchors;
    Pose& ref = anchors.reference;

    double prev_x;
    double prev_y;

    auto prev_path_size = telemetry.last_trajectory.x.size();
    if (prev_path_size < 2) {
      // based only on car
      ref.yaw = telemetry.car_yaw;
      ref.x = telemetry.car_x;
      ref.y = telemetry.car_y;
      ref.s = telemetry.car_s;
      ref.d = telemetry.car_d;
      prev_x = ref.x - cos(ref.yaw);
      prev_y = ref.y - sin(ref.yaw);
    } else {
      // based only on previous endpoints
      prev_x = telemetry.last_trajectory.x[prev_path_size - 2];
      prev_y = telemetry.last_trajectory.y[prev_path_size - 2];
      ref.x = telemetry.last_trajectory.x[prev_path_size - 1];
      ref.y = telemetry.last_trajectory.y[prev_path_size - 1];
      ref.yaw = atan2(ref.y - prev_y, ref.x - prev_x);
      ref.s = telemetry.end_path_s;
      ref.d = telemetry.car_d;
    }
    anchors.x = {prev_x, ref.x};
    anchors.y = {prev_y, ref.y};

    // Add extra anchors
    int n_missing_anchors = n_anchors_ - 2;
    for (int i = 0; i < n_missing_anchors; ++i) {
      float spacing = look_ahead_distance_ / n_missing_anchors;
      float s = ref.s + (i + 1) * spacing;
      float d = lane_width_ * (0.5 + target_lane_id);
      std::vector<double> anchor =
          getXY(s, d, map_->waypoints_s, map_->waypoints_x, map_->waypoints_y);
      anchors.x.push_back(anchor[0]);
      anchors.y.push_back(anchor[1]);
    }
    return anchors;
  }

  /**
   * Transform anchors to ego frame to ensure they are as horizontal as
   * possible, so that for any given x, there is only one y value.
   */
  SplineAnchors transformToEgo(const SplineAnchors& anchors) {
    SplineAnchors result;
    const Pose& ref = anchors.reference;
    result.reference = anchors.reference;
    result.x.resize(n_anchors_);
    result.y.resize(n_anchors_);

    for (int i = 0; i < n_anchors_; i++) {
      double shift_x = anchors.x[i] - ref.x;
      double shift_y = anchors.y[i] - ref.y;
      result.x[i] = shift_x * cos(0 - ref.yaw) - shift_y * sin(0 - ref.yaw);
      result.y[i] = shift_x * sin(0 - ref.yaw) + shift_y * cos(0 - ref.yaw);
    }
    return result;
  }

  Point transformToMap(const Point& p, const Pose& ref) {
    Point result;
    result.x = p.x * cos(ref.yaw) - p.y * sin(ref.yaw);
    result.y = p.x * sin(ref.yaw) + p.y * cos(ref.yaw);
    result.x += ref.x;
    result.y += ref.y;
    return result;
  }

  /**
   * Generate interpolation points between anchors.
   * Points are evenly spaced in x axis, so that desired speed is kept.
   */
  Trajectory interpolateMissingPoints(const Trajectory& baseline,
                                      const SplineAnchors& ego_anchors) {
    Trajectory result = baseline;
    int missing_points = path_size_ - baseline.x.size();

    tk::spline gen;
    gen.set_points(ego_anchors.x, ego_anchors.y);

    double target_x = look_ahead_distance_;
    double target_y = gen(target_x);
    double target_dist = distance(0, 0, target_x, target_y);
    double N = target_dist * path_execution_frequency_ / current_velocity_mps_;

    for (int i = 0; i < missing_points; ++i) {
      Point ego_point;
      ego_point.x = (i + 1) * target_x / N;
      ego_point.y = gen(ego_point.x);

      Point map_point = transformToMap(ego_point, ego_anchors.reference);
      result.x.push_back(map_point.x);
      result.y.push_back(map_point.y);
    }
    return result;
  }

  bool isObjectInLane(const FusedObject& object, std::uint8_t lane_id) {
    double center_lane_d = lane_width_ * (0.5 + lane_id);
    double left_boundary_d = center_lane_d - lane_width_ / 2.0;
    double right_boundary_d = center_lane_d + lane_width_ / 2.0;
    return left_boundary_d < object.d && object.d < right_boundary_d;
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
  Trajectory generateKeepLaneTrajectory(const TelemetryPacket& telemetry,
                                        const PredictionData& predictions) {
    SplineAnchors anchors = generateSplineAnchors(telemetry, current_lane_id_);
    SplineAnchors ego_anchors = transformToEgo(anchors);

    int prev_size = telemetry.last_trajectory.x.size();
    bool too_close = false;
    for (const auto object : predictions.sensor_fusion) {
      if (isObjectInLane(object, current_lane_id_)) {
        double speed = sqrt(object.vx * object.vx + object.vy * object.vy);
        double object_s = object.s;

        // If using the previous path, then we are not yet there??
        // Then we project the object in time to be prev_size steps ahead.
        if (prev_size > 0) {
          object_s += prev_size * speed / path_execution_frequency_;
        }

        double car_s = anchors.reference.s;
        double safety_gap = 30.0;
        bool is_object_in_front = car_s < object_s;
        bool is_object_near = abs(car_s - object_s) < safety_gap;
        if (is_object_in_front && is_object_near) {
          too_close = true;
          break;
        }
      }
    }
    if (too_close) {
      // TODO: decrease velocity according to computed deceleration
      current_velocity_mps_ -= velocity_delta_mps_;
    } else if (current_velocity_mps_ < target_velocity_mps_) {
      // TODO: increment velocity according to acceleration
      current_velocity_mps_ += velocity_delta_mps_;
    }
    // std::cout << "speed: " << current_velocity_mps_ << std::endl;

    return interpolateMissingPoints(telemetry.last_trajectory, ego_anchors);
  }

  Trajectory generateLaneChangeLeftTrajectory(const TelemetryPacket& telemetry,
                                              const PredictionData&) {
    std::uint8_t target_lane_id = fmax(0, current_lane_id_ - 1);
    SplineAnchors anchors = generateSplineAnchors(telemetry, target_lane_id);
    SplineAnchors ego_anchors = transformToEgo(anchors);
    return interpolateMissingPoints(telemetry.last_trajectory, ego_anchors);
  }

  Trajectory generateLaneChangeRightTrajectory(const TelemetryPacket& telemetry,
                                               const PredictionData&) {
    std::uint8_t target_lane_id = fmin(2, current_lane_id_ + 1);
    SplineAnchors anchors = generateSplineAnchors(telemetry, target_lane_id);
    SplineAnchors ego_anchors = transformToEgo(anchors);
    return interpolateMissingPoints(telemetry.last_trajectory, ego_anchors);
  }

 private:
  // Input Data
  std::shared_ptr<Map> map_;
  TelemetryPacket telemetry_;

  // environment
  float lane_width_{4.0F};

  // target
  std::uint8_t current_lane_id_{1};  // 0=left, 1=middle, 2=right
  float current_velocity_mps_{0.0F};
  float target_velocity_mph_{49.5F};
  float target_velocity_mps_{target_velocity_mph_ / 2.237F};

  // algorithms
  int path_size_{50};
  int n_anchors_{5};
  float path_execution_frequency_{1 / .02F};
  float look_ahead_distance_{90.0F};
  float velocity_delta_mps_{0.1F};
};

}  // namespace udacity

#endif  // TRAJECTORY_GENERATOR_H_

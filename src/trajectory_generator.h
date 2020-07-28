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
  }
  void setEgoStatus(const EgoStatus& ego) { ego_ = ego; }

  Trajectory getTrajectoryForAction(TrajectoryAction action,
                                    const PredictionData& predictions) {
    Trajectory result;
    switch (action) {
      case TrajectoryAction::kKeepLane:
        result = generateKeepLaneTrajectory(predictions);
        break;
      case TrajectoryAction::kPrepareChangeLaneLeft:
      case TrajectoryAction::kChangeLaneLeft:
        result = generateLaneChangeLeftTrajectory(predictions);
        break;
      case TrajectoryAction::kPrepareChangeLaneRight:
      case TrajectoryAction::kChangeLaneRight:
        result = generateLaneChangeRightTrajectory(predictions);
        break;
    }
    result.characteristics.action = action;
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
  SplineAnchors generateSplineAnchors(std::uint8_t intended_lane_id) {
    SplineAnchors anchors;
    Pose& ref = anchors.reference;

    double prev_x;
    double prev_y;

    auto prev_path_size = telemetry_.last_trajectory.x.size();
    if (prev_path_size < 2) {
      // based only on car
      ref.yaw = telemetry_.car_yaw;
      ref.x = telemetry_.car_x;
      ref.y = telemetry_.car_y;
      ref.s = telemetry_.car_s;
      ref.d = telemetry_.car_d;
      prev_x = ref.x - cos(ref.yaw);
      prev_y = ref.y - sin(ref.yaw);
    } else {
      // based only on previous endpoints
      prev_x = telemetry_.last_trajectory.x[prev_path_size - 2];
      prev_y = telemetry_.last_trajectory.y[prev_path_size - 2];
      ref.x = telemetry_.last_trajectory.x[prev_path_size - 1];
      ref.y = telemetry_.last_trajectory.y[prev_path_size - 1];
      ref.yaw = atan2(ref.y - prev_y, ref.x - prev_x);
      ref.s = telemetry_.end_path_s;
      ref.d = telemetry_.car_d;
    }
    anchors.x = {prev_x, ref.x};
    anchors.y = {prev_y, ref.y};

    // Add extra anchors
    int n_missing_anchors = n_anchors_ - 2;
    for (int i = 0; i < n_missing_anchors; ++i) {
      float spacing = look_ahead_distance_ / n_missing_anchors;
      float s = ref.s + (i + 1) * spacing;
      float d = environment_.lane_width * (0.5 + intended_lane_id);
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
                                      const SplineAnchors& ego_anchors,
                                      double speed) {
    Trajectory result = baseline;
    int missing_points = path_size_ - baseline.x.size();

    tk::spline gen;
    gen.set_points(ego_anchors.x, ego_anchors.y);

    double target_x = look_ahead_distance_;
    double target_y = gen(target_x);
    double target_dist = distance(0, 0, target_x, target_y);
    double N = target_dist * path_execution_frequency_ / speed;

    for (int i = 0; i < missing_points; ++i) {
      Point ego_point;
      ego_point.x = (i + 1) * target_x / N;
      ego_point.y = gen(ego_point.x);

      Point map_point = transformToMap(ego_point, ego_anchors.reference);
      result.x.push_back(map_point.x);
      result.y.push_back(map_point.y);
    }
    result.characteristics.speed = speed;
    return result;
  }

  bool isObjectInLane(const FusedObject& object, std::uint8_t lane_id) {
    double lane_width = environment_.lane_width;
    double center_lane_d = lane_width * (0.5 + lane_id);
    double left_boundary_d = center_lane_d - lane_width / 2.0;
    double right_boundary_d = center_lane_d + lane_width / 2.0;
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
  Trajectory generateKeepLaneTrajectory(const PredictionData& predictions) {
    SplineAnchors anchors = generateSplineAnchors(ego_.lane_id);
    SplineAnchors ego_anchors = transformToEgo(anchors);

    int prev_size = telemetry_.last_trajectory.x.size();
    bool too_close = false;
    for (const auto object : predictions.sensor_fusion) {
      if (isObjectInLane(object, ego_.lane_id)) {
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

    double speed = getSpeedForecast();
    if (too_close) {
      speed = ego_.speed - speed_brake_delta_mps_;
    }
    Trajectory trajectory = interpolateMissingPoints(telemetry_.last_trajectory,
                                                     ego_anchors, speed);
    trajectory.characteristics.intended_lane_id = ego_.lane_id;
    return trajectory;
  }

  double getSpeedForecast() {
    // TODO: update speed according to acceleration/braking
    // TODO: speed from telemetry is not reliable. Why?
    // double speed = telemetry.car_speed;
    double speed = ego_.speed;
    if (speed < target_.speed) {
      speed += speed_control_delta_mps_;
    } else {
      speed -= speed_control_delta_mps_;
    }
    return speed;
  }

  Trajectory generateLaneChangeLeftTrajectory(const PredictionData&) {
    std::uint8_t intended_lane_id = fmax(0, ego_.lane_id - 1);
    SplineAnchors anchors = generateSplineAnchors(intended_lane_id);
    SplineAnchors ego_anchors = transformToEgo(anchors);
    Trajectory trajectory = interpolateMissingPoints(telemetry_.last_trajectory,
                                                     ego_anchors, ego_.speed);
    trajectory.characteristics.intended_lane_id = intended_lane_id;
    return trajectory;
  }

  Trajectory generateLaneChangeRightTrajectory(const PredictionData&) {
    std::uint8_t intended_lane_id = fmin(2, ego_.lane_id + 1);
    SplineAnchors anchors = generateSplineAnchors(intended_lane_id);
    SplineAnchors ego_anchors = transformToEgo(anchors);
    Trajectory trajectory = interpolateMissingPoints(telemetry_.last_trajectory,
                                                     ego_anchors, ego_.speed);
    trajectory.characteristics.intended_lane_id = intended_lane_id;
    return trajectory;
  }

 private:
  // Input Data
  std::shared_ptr<Map> map_;
  TelemetryPacket telemetry_;

  EnvironmentData environment_;
  TargetData target_;
  EgoStatus ego_;

  // algorithms
  int path_size_{50};
  int n_anchors_{5};
  float path_execution_frequency_{1 / .02F};
  float look_ahead_distance_{90.0F};

  // speed
  float speed_control_delta_mps_{0.1F};
  float speed_brake_delta_mps_{0.3F};
};  // namespace udacity

}  // namespace udacity

#endif  // TRAJECTORY_GENERATOR_H_

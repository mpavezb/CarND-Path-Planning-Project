#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <vector>

#include "environment.h"
#include "helpers.h"
#include "spline.h"

namespace udacity {

class MotionPlanner {
 public:
  MotionPlanner(const Map& map) : map_(map) {}

  /**
   * Generate spline anchors for interpolation.
   *
   * Anchors are based on the previous path's endpoint whenever possible.
   * Otherwise, the car is used as starting reference.
   *
   * Note that previous path end point is always further away than the car.
   */
  SplineAnchors generateSplineAnchors(const TelemetryPacket& telemetry) {
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
      prev_x = ref.x - cos(ref.yaw);
      prev_y = ref.y - sin(ref.yaw);
    } else {
      // based only on previous endpoints
      prev_x = telemetry.last_trajectory.x[prev_path_size - 2];
      prev_y = telemetry.last_trajectory.y[prev_path_size - 2];
      ref.x = telemetry.last_trajectory.x[prev_path_size - 1];
      ref.y = telemetry.last_trajectory.y[prev_path_size - 1];
      ref.yaw = atan2(ref.y - prev_y, ref.x - prev_x);
    }
    anchors.x = {prev_x, ref.x};
    anchors.y = {prev_y, ref.y};

    // Add extra anchors
    int n_missing_anchors = n_anchors_ - 2;
    for (int i = 0; i < n_missing_anchors; ++i) {
      float spacing = look_ahead_distance_ / n_missing_anchors;
      float s = telemetry.car_s + (i + 1) * spacing;
      float d = lane_width_ * (0.5 + target_lane_id_);
      std::vector<double> anchor =
          getXY(s, d, map_.waypoints_s, map_.waypoints_x, map_.waypoints_y);
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
    double N = target_dist * path_execution_frequency_ / target_velocity_mps_;

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
  Trajectory generateTrajectory(const TelemetryPacket& telemetry) {
    SplineAnchors anchors = generateSplineAnchors(telemetry);
    SplineAnchors ego_anchors = transformToEgo(anchors);
    return interpolateMissingPoints(telemetry.last_trajectory, ego_anchors);
  }

 private:
  // environment
  Map map_;
  float lane_width_{4.0F};

  // target
  std::uint8_t target_lane_id_{1};  // 1=left, 2=middle, 3=right
  float target_velocity_mph_{49.5F};
  float target_velocity_mps_{49.5F / 2.24F};

  // algorithms
  int path_size_{50};
  int n_anchors_ = 5;
  float path_execution_frequency_{1 / .02F};
  float look_ahead_distance_ = 90;
};

}  // namespace udacity

#endif  // PLANNER_H

#ifndef PLANNER_H
#define PLANNER_H

#include <vector>

#include "environment.h"
#include "helpers.h"
#include "spline.h"

namespace udacity {

struct SplineAnchors {
  Path x;
  Path y;
  double ref_x;
  double ref_y;
  double ref_x_prev;
  double ref_y_prev;
  double ref_yaw;
};

class MotionPlanner {
 public:
  MotionPlanner(const Map& map) : map_(map) {}

  /**
   * Generate spline anchors for interpolation.
   *
   * Anchors are based on the previous path's endpoint whenever possible.
   * Otherwise, the car is used as starting reference.
   */
  SplineAnchors generateSplineAnchors(const TelemetryPacket& telemetry) {
    SplineAnchors anchors;

    auto prev_path_size = telemetry.previous_path_x.size();
    if (prev_path_size < 2) {
      anchors.ref_yaw = telemetry.car_yaw;
      anchors.ref_x = telemetry.car_x;
      anchors.ref_y = telemetry.car_y;
      anchors.ref_x_prev = anchors.ref_x - cos(anchors.ref_yaw);
      anchors.ref_y_prev = anchors.ref_y - sin(anchors.ref_yaw);
    } else {
      anchors.ref_x = telemetry.previous_path_x[prev_path_size - 1];
      anchors.ref_y = telemetry.previous_path_y[prev_path_size - 1];
      anchors.ref_x_prev = telemetry.previous_path_x[prev_path_size - 2];
      anchors.ref_y_prev = telemetry.previous_path_y[prev_path_size - 2];
      anchors.ref_yaw = atan2(anchors.ref_y - anchors.ref_y_prev,
                              anchors.ref_x - anchors.ref_x_prev);
    }
    anchors.x = {anchors.ref_x_prev, anchors.ref_x};
    anchors.y = {anchors.ref_y_prev, anchors.ref_y};

    // Add extra anchors
    // TODO: 4 = lane width?
    int n_extra_anchors = n_anchors_ - 2;
    for (int i = 0; i < n_extra_anchors; ++i) {
      float spacing = look_ahead_distance_ / n_extra_anchors;
      std::vector<double> anchor =
          getXY(telemetry.car_s + spacing, (2 + 4 * target_lane_id_),
                map_.waypoints_s, map_.waypoints_x, map_.waypoints_y);
      anchors.x.push_back(anchor[0]);
      anchors.y.push_back(anchor[1]);
    }
    return anchors;
  }

  /**
   * Spline based Trajectory Generation.
   *
   * Anchor points are generated based on the current state and the goal state.
   * Continuity is ensured by using the previously generated path as a baseline
   * and a spline as interpolator for next path points.
   *
   * Paths are expected to have a fixed length. The generator appends missing
   * points to the previously generated trajectory.
   */
  void generateTrajectory(const TelemetryPacket& telemetry) {
    next_x_vals.clear();
    next_y_vals.clear();

    // Reuse what remains from previous path
    next_x_vals = telemetry.previous_path_x;
    next_y_vals = telemetry.previous_path_y;

    SplineAnchors anchors = generateSplineAnchors(telemetry);

    // Transform interpolation path to ego frame
    // This is important to ensure the path is as horizontal as possible,
    // so that for any given x, there is only one y value.

    for (int i = 0; i < 5; i++) {
      // shift + rotation
      double shift_x = anchors.x[i] - anchors.ref_x;
      double shift_y = anchors.y[i] - anchors.ref_y;
      anchors.x[i] = shift_x * cos(0 - anchors.ref_yaw) -
                     shift_y * sin(0 - anchors.ref_yaw);
      anchors.y[i] = shift_x * sin(0 - anchors.ref_yaw) +
                     shift_y * cos(0 - anchors.ref_yaw);
    }

    tk::spline s;
    s.set_points(anchors.x, anchors.y);
  }

  Path getNextXVals() const { return next_x_vals; }
  Path getNextYVals() const { return next_y_vals; }

 private:
  Path next_x_vals;
  Path next_y_vals;
  std::uint8_t target_lane_id_{1};
  float target_velocity_mph_{49.5F};
  int path_size_{50};
  float path_execution_frequency_{1 / .02F};
  int n_anchors_ = 5;
  float look_ahead_distance_ = 90;  // [m]
  Map map_;
};

}  // namespace udacity

#endif  // PLANNER_H

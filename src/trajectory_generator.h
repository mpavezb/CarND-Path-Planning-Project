#ifndef TRAJECTORY_GENERATOR_H_
#define TRAJECTORY_GENERATOR_H_

#include <memory>
#include <vector>

#include "data_types.h"
#include "helpers.h"
#include "parameters.h"
#include "third_party/spline.h"

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

  void setPredictions(const PredictionData& predictions) {
    predictions_ = predictions;
  }

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

  Trajectory getTrajectoryForAction(TrajectoryAction action) const {
    Trajectory result;
    switch (action) {
      case TrajectoryAction::kKeepLane:
        result = generateKeepLaneTrajectory();
        break;
      case TrajectoryAction::kPrepareChangeLaneLeft:
        result = generatePrepareLaneChangeLeftTrajectory();
        break;
      case TrajectoryAction::kChangeLaneLeft:
        result = generateLaneChangeLeftTrajectory();
        break;
      case TrajectoryAction::kPrepareChangeLaneRight:
        result = generatePrepareLaneChangeRightTrajectory();
        break;
      case TrajectoryAction::kChangeLaneRight:
        result = generateLaneChangeRightTrajectory();
        break;
    }
    result.characteristics.action = action;
    return result;
  }

  /**
   * Generate spline anchors for interpolation.
   */
  SplineAnchors generateSplineAnchors(std::uint8_t intended_lane_id) const {
    SplineAnchors anchors;
    const AnchorReference& ref = anchor_reference_;
    anchors.push_back({ref.x1, ref.y1});
    anchors.push_back({ref.x2, ref.y2});

    // Add extra anchors
    const int n_missing_anchors = parameters_.n_anchors_ - 2;
    for (int i = 0; i < n_missing_anchors; ++i) {
      float spacing =
          parameters_.anchors_look_ahead_distance / n_missing_anchors;
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
  SplineAnchors transformToEgo(const SplineAnchors& anchors) const {
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

  Point transformToMap(const Point& p) const {
    Point result;
    const AnchorReference& ref = anchor_reference_;
    result.x = p.x * cos(ref.yaw) - p.y * sin(ref.yaw);
    result.y = p.x * sin(ref.yaw) + p.y * cos(ref.yaw);
    result.x += ref.x2;
    result.y += ref.y2;
    return result;
  }

  bool areAnchorsValid(const SplineAnchors& ego_anchors) const {
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
                                      double speed) const {
    Trajectory result;
    result.path = telemetry_.last_path;
    int missing_points = parameters_.path_size_ - result.path.size();

    if (not areAnchorsValid(ego_anchors)) {
      return generateInvalidTrajectory();
    }

    tk::spline gen;
    gen.set_points(getPathX(ego_anchors), getPathY(ego_anchors));

    double target_x = parameters_.trajectory_length;
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

  double getLimitedSpeed(double target_speed) const {
    // TODO: this helps avoiding invalid anchor generation
    // if speed is near zero, then last trajectory points
    // will be too near for anchors to be valid.
    const double min_speed = 0.1;
    const double max_speed = parameters_.desired_speed;

    // Increase/Decrease as much as needed, limited by acceleration
    double delta_speed;
    double diff_to_target = target_speed - ego_.speed;
    if (diff_to_target > 0) {
      delta_speed = fmin(diff_to_target, parameters_.acceleration);
    } else {
      delta_speed = -1.0 * fmin(fabs(diff_to_target), parameters_.deceleration);
    }
    return fmax(min_speed, fmin(ego_.speed + delta_speed, max_speed));
  }

  double getKeepLaneSpeed(std::uint8_t lane_id) const {
    const auto& lane = predictions_.lanes[lane_id];
    // TODO: Need to know when to start stopping to avoid colliding front
    // vehicle!
    // TODO: Dont increase too much if there is a vehicle ahead. Keep speed
    // instead. This will result in a sudden speed increase.
    // Keep max velocity possible

    // Default case: Go to max speed.
    double target_speed = parameters_.desired_speed;

    bool has_ahead = lane.has_vehicle_ahead and lane.vehicle_ahead.is_near;
    bool has_behind = lane.has_vehicle_behind and lane.vehicle_behind.is_near;
    if (has_ahead) {
      if (has_behind) {
        // keep speed of slower vehicle
        target_speed =
            fmin(lane.vehicle_ahead.speed, lane.vehicle_behind.speed);
      } else {
        // slow down as much as needed to match speeds
        target_speed = lane.vehicle_ahead.speed;
      }
    }
    return getLimitedSpeed(target_speed);
  }

  double getPrepareLaneChangeSpeed(std::uint8_t intended_lane_id,
                                   std::uint8_t endpoint_lane_id) const {
    const auto& intended_lane = predictions_.lanes[intended_lane_id];
    const auto& endpoint_lane = predictions_.lanes[endpoint_lane_id];

    const double keep_lane_speed = getKeepLaneSpeed(endpoint_lane_id);
    const double next_lane_speed = getKeepLaneSpeed(intended_lane_id);

    const bool has_ahead_curr =
        endpoint_lane.has_vehicle_ahead and endpoint_lane.vehicle_ahead.is_near;
    const bool has_ahead_next =
        intended_lane.has_vehicle_ahead and intended_lane.vehicle_ahead.is_near;
    const bool has_behind_curr = endpoint_lane.has_vehicle_behind and
                                 endpoint_lane.vehicle_behind.is_near;
    const bool has_behind_next = intended_lane.has_vehicle_behind and
                                 intended_lane.vehicle_behind.is_near;

    // go to the speed of the intended lane
    double target_speed = intended_lane.speed;

    // limit target speed according to current lane restrictions
    // - dont exceed keep_lane_velocity
    // - dont collide with car ahead and behind
    double speed_limit_max = keep_lane_speed;
    double speed_limit_min = 0.0;
    if (has_behind_curr) {
      speed_limit_min =
          fmax(speed_limit_min, endpoint_lane.vehicle_behind.speed);
    }
    if (has_ahead_curr) {
      speed_limit_max =
          fmin(speed_limit_max, endpoint_lane.vehicle_ahead.speed);
    }
    target_speed = fmin(speed_limit_max, fmax(speed_limit_min, target_speed));
    return getLimitedSpeed(keep_lane_speed);
  }

  double getLaneChangeSpeed(std::uint8_t intended_lane_id,
                            std::uint8_t endpoint_lane_id) const {
    return getKeepLaneSpeed(intended_lane_id);
  }

  bool isLaneChangePossible(std::uint8_t intended_lane_id) const {
    const auto& lane = predictions_.lanes[intended_lane_id];
    for (const auto& vehicle : lane.vehicles) {
      if ((vehicle.is_ahead or vehicle.is_behind) and
          vehicle.predicted_distance < parameters_.safe_distance) {
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
                                          double speed) const {
    // Generate spline
    SplineAnchors anchors = generateSplineAnchors(endpoint_lane_id);
    SplineAnchors ego_anchors = transformToEgo(anchors);
    Trajectory trajectory = interpolateMissingPoints(ego_anchors, speed);
    trajectory.characteristics.endpoint_lane_id = endpoint_lane_id;
    trajectory.characteristics.intended_lane_id = intended_lane_id;
    return trajectory;
  }

  Trajectory generateInvalidTrajectory() const {
    Trajectory invalid_trajectory;
    invalid_trajectory.characteristics.is_valid = false;
    return invalid_trajectory;
  }

  Trajectory generateTrajectory(std::uint8_t intended_lane_id,
                                std::uint8_t endpoint_lane_id,
                                double speed) const {
    return generateTrajectoryFromSpline(intended_lane_id, endpoint_lane_id,
                                        speed);
  }

  Trajectory generateKeepLaneTrajectory() const {
    std::uint8_t intended_lane_id = ego_.lane_id;
    std::uint8_t endpoint_lane_id = ego_.lane_id;
    double speed = getKeepLaneSpeed(intended_lane_id);
    return generateTrajectory(intended_lane_id, endpoint_lane_id, speed);
  }

  Trajectory generatePrepareLaneChangeTrajectory(
      std::uint8_t intended_lane_id, std::uint8_t endpoint_lane_id) const {
    double speed =
        getPrepareLaneChangeSpeed(intended_lane_id, endpoint_lane_id);
    return generateTrajectory(intended_lane_id, endpoint_lane_id, speed);
  }

  Trajectory generateLaneChangeTrajectory(std::uint8_t intended_lane_id,
                                          std::uint8_t endpoint_lane_id) const {
    if (!isLaneChangePossible(intended_lane_id)) {
      return generateInvalidTrajectory();
    }
    double speed = getLaneChangeSpeed(intended_lane_id, endpoint_lane_id);
    return generateTrajectory(intended_lane_id, endpoint_lane_id, speed);
  }

  Trajectory generatePrepareLaneChangeLeftTrajectory() const {
    std::uint8_t intended_lane_id = fmax(0, ego_.lane_id - 1);
    std::uint8_t endpoint_lane_id = ego_.lane_id;
    return generatePrepareLaneChangeTrajectory(intended_lane_id,
                                               endpoint_lane_id);
  }

  Trajectory generatePrepareLaneChangeRightTrajectory() const {
    std::uint8_t intended_lane_id = fmin(2, ego_.lane_id + 1);
    std::uint8_t endpoint_lane_id = ego_.lane_id;
    return generatePrepareLaneChangeTrajectory(intended_lane_id,
                                               endpoint_lane_id);
  }

  Trajectory generateLaneChangeLeftTrajectory() const {
    std::uint8_t intended_lane_id = fmax(0, ego_.lane_id - 1);
    std::uint8_t endpoint_lane_id = intended_lane_id;
    return generateLaneChangeTrajectory(intended_lane_id, endpoint_lane_id);
  }

  Trajectory generateLaneChangeRightTrajectory() const {
    std::uint8_t intended_lane_id = fmin(2, ego_.lane_id + 1);
    std::uint8_t endpoint_lane_id = intended_lane_id;
    return generateLaneChangeTrajectory(intended_lane_id, endpoint_lane_id);
  }

 private:
  // Input Data
  std::shared_ptr<Map> map_;
  Parameters parameters_;
  TelemetryPacket telemetry_;
  EgoStatus ego_;
  PredictionData predictions_;
  AnchorReference anchor_reference_;
};

}  // namespace udacity

#endif  // TRAJECTORY_GENERATOR_H_

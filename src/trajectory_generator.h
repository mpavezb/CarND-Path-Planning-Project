#ifndef TRAJECTORY_GENERATOR_H_
#define TRAJECTORY_GENERATOR_H_

#include <algorithm>
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
    auto last_path_size =
        fmin(parameters_.previous_path_keep, telemetry_.last_path.size());
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

      auto frenet = getFrenet(ref.x2, ref.y2, ref.yaw, map_);
      ref.s = frenet.s;
      ref.d = frenet.d;
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
    const int n_missing_anchors = parameters_.n_anchors - 2;
    for (int i = 0; i < n_missing_anchors; ++i) {
      float spacing =
          parameters_.anchors_look_ahead_distance / n_missing_anchors;
      float s = ref.s + (i + 1) * spacing;
      float d = parameters_.lane_width * (0.5 + intended_lane_id);
      Point anchor = getXY(s, d, map_);
      anchors.push_back(anchor);
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
    result.resize(parameters_.n_anchors);

    for (int i = 0; i < parameters_.n_anchors; i++) {
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
    auto points_to_keep =
        fmin(parameters_.previous_path_keep, telemetry_.last_path.size());

    for (int i = 0; i < points_to_keep; ++i) {
      result.path.push_back(telemetry_.last_path[i]);
    }
    int missing_points = parameters_.path_size - result.path.size();

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
    result.frenet_path = getFrenetPath(result.path, telemetry_.car_yaw, map_);
    result.characteristics.speed = speed;
    result.characteristics.is_valid = true;

    return result;
  }

  double getLimitedSpeedByMinMax(double target_speed) const {
    // Min speed = 0.1 helps avoiding invalid anchor generation.
    const double min_speed = 0.1;
    const double max_speed = parameters_.desired_speed;
    return fmax(min_speed, fmin(target_speed, max_speed));
  }

  double getLimitedSpeedByAcceleration(double target_speed) const {
    double delta_speed;
    double diff_to_target = target_speed - ego_.speed;
    if (diff_to_target > 0) {
      delta_speed = fmin(diff_to_target, parameters_.max_acceleration);
    } else {
      delta_speed =
          -1.0 * fmin(fabs(diff_to_target), parameters_.max_deceleration);
    }
    return ego_.speed + delta_speed;
  }

  /**
   * If there is a car ahead, keep a slightly slower speed, to make some
   * distance. If there is also a car behind, then keep speed from slowest one.
   */
  double getLimitedSpeedByCollisions(double target_speed,
                                     std::uint8_t lane_id) const {
    const auto& lane = predictions_.lanes[lane_id];
    bool has_ahead = lane.has_vehicle_ahead and lane.vehicle_ahead.is_near;
    bool has_behind = lane.has_vehicle_behind and lane.vehicle_behind.is_near;

    double safe_speed = target_speed;
    if (has_ahead) {
      if (has_behind) {
        safe_speed = fmin(lane.vehicle_ahead.speed, lane.vehicle_behind.speed);
      } else {
        safe_speed =
            lane.vehicle_ahead.speed - parameters_.keep_distance_delta_speed;
      }
    }
    return fmin(target_speed, safe_speed);
  }

  /**
   * Attempt going to the max possible speed, but adecuate to avoid collisions.
   */
  double getKeepLaneSpeed(std::uint8_t lane_id) const {
    double target_speed = parameters_.desired_speed;
    target_speed = getLimitedSpeedByCollisions(target_speed, lane_id);
    target_speed = getLimitedSpeedByAcceleration(target_speed);
    target_speed = getLimitedSpeedByMinMax(target_speed);
    return target_speed;
  }

  /**
   * If lanes are free, attemp keeping max speed.
   * If there is a vehicle in the intended lane, reduce speed to create gap.
   * Then adecuate to avoid collisions on current lane.
   */
  double getPrepareLaneChangeSpeed(std::uint8_t intended_lane_id,
                                   std::uint8_t endpoint_lane_id) const {
    const auto& intended_lane = predictions_.lanes[intended_lane_id];
    double target_speed = parameters_.desired_speed;
    if (intended_lane.has_vehicle_ahead and
        intended_lane.vehicle_ahead.is_near) {
      target_speed =
          intended_lane.vehicle_ahead.speed - parameters_.max_deceleration;
    }
    target_speed = getLimitedSpeedByCollisions(target_speed, endpoint_lane_id);
    target_speed = getLimitedSpeedByAcceleration(target_speed);
    target_speed = getLimitedSpeedByMinMax(target_speed);
    return target_speed;
  }

  /**
   * Attempt going a little faster than the prepare for lane change speed, but
   * adecuate to avoid collisions on both lanes.
   */
  double getLaneChangeSpeed(std::uint8_t intended_lane_id,
                            std::uint8_t endpoint_lane_id) const {
    double target_speed =
        getPrepareLaneChangeSpeed(intended_lane_id, endpoint_lane_id) +
        parameters_.max_acceleration;
    target_speed = getLimitedSpeedByCollisions(target_speed, endpoint_lane_id);
    target_speed = getLimitedSpeedByCollisions(target_speed, intended_lane_id);
    target_speed = getLimitedSpeedByAcceleration(target_speed);
    target_speed = getLimitedSpeedByMinMax(target_speed);
    return target_speed;
  }

  bool isLaneChangePossible(std::uint8_t intended_lane_id) const {
    const auto& lane = predictions_.lanes[intended_lane_id];
    for (const auto& vehicle : lane.vehicles) {
      if (vehicle.is_ahead and
          vehicle.predicted_distance < parameters_.safe_distance_ahead) {
        return false;
      }
      if (vehicle.is_behind and
          vehicle.predicted_distance < parameters_.safe_distance_behind) {
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

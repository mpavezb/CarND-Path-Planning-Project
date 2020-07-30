#ifndef TRAJECTORY_GENERATOR_H_
#define TRAJECTORY_GENERATOR_H_

#include <iostream>
#include <memory>
#include <vector>

#include "data_types.h"
#include "helpers.h"
#include "third_party/spline.h"

namespace udacity {

/**
 * Provides trajectories for different scenarios, based on the optimal speed for
 * it.
 */
class TrajectoryGenerator {
 public:
  void setMap(std::shared_ptr<Map> map) { map_ = map; }
  void setTelemetry(const TelemetryPacket& telemetry) {
    telemetry_ = telemetry;
  }
  void setEgoStatus(const EgoStatus& ego) { ego_ = ego; }

  AnchorReference anchor_reference_;

  /**
   * Anchors are based on the previous path's endpoint whenever possible.
   * Otherwise, the car is used as starting reference.
   *
   * Note that previous path end point is always further away than the car.
   */
  void updateAnchorReference() {
    AnchorReference& ref = anchor_reference_;
    auto prev_path_size = telemetry_.last_trajectory.x.size();
    if (prev_path_size < 2) {
      // based only on car
      ref.yaw = telemetry_.car_yaw;
      ref.x2 = telemetry_.car_x;
      ref.y2 = telemetry_.car_y;
      ref.s = telemetry_.car_s;
      ref.d = telemetry_.car_d;
      ref.x1 = ref.x2 - cos(ref.yaw);
      ref.y1 = ref.y2 - sin(ref.yaw);
      // std::cout << "[generateSplineAnchors] based on car" << std::endl;
    } else {
      // based only on previous endpoints
      ref.x1 = telemetry_.last_trajectory.x[prev_path_size - 2];
      ref.y1 = telemetry_.last_trajectory.y[prev_path_size - 2];
      ref.x2 = telemetry_.last_trajectory.x[prev_path_size - 1];
      ref.y2 = telemetry_.last_trajectory.y[prev_path_size - 1];
      ref.yaw = atan2(ref.y2 - ref.y1, ref.x2 - ref.x1);
      ref.s = telemetry_.end_path_s;
      ref.d = telemetry_.car_d;
      // std::cout << "[generateSplineAnchors] based on previous" << std::endl;
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
    anchors.x = {ref.x1, ref.x2};
    anchors.y = {ref.y1, ref.y2};

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
    const AnchorReference& ref = anchor_reference_;
    result.x.resize(n_anchors_);
    result.y.resize(n_anchors_);

    for (int i = 0; i < n_anchors_; i++) {
      double shift_x = anchors.x[i] - ref.x2;
      double shift_y = anchors.y[i] - ref.y2;
      result.x[i] = shift_x * cos(0 - ref.yaw) - shift_y * sin(0 - ref.yaw);
      result.y[i] = shift_x * sin(0 - ref.yaw) + shift_y * cos(0 - ref.yaw);
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

  /**
   * Generate interpolation points between anchors.
   * Points are evenly spaced in x axis, so that desired speed is kept.
   */
  Trajectory interpolateMissingPoints(const SplineAnchors& ego_anchors,
                                      double speed) {
    Trajectory result = telemetry_.last_trajectory;
    int missing_points = path_size_ - result.x.size();

    tk::spline gen;
    gen.set_points(ego_anchors.x, ego_anchors.y);

    double target_x = look_ahead_distance_;
    double target_y = gen(target_x);
    double target_dist = distance(0, 0, target_x, target_y);
    double N = target_dist / (speed * environment_.time_step_);

    for (int i = 0; i < missing_points; ++i) {
      Point ego_point;
      ego_point.x = (i + 1) * target_x / N;
      ego_point.y = gen(ego_point.x);

      Point map_point = transformToMap(ego_point);
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

  double predictObjectPosition(const FusedObject& object) {
    double s = object.s;
    double speed = sqrt(object.vx * object.vx + object.vy * object.vy);
    // If using the previous path, then we are not yet there??
    // Then we project the object in time to be prev_size steps ahead.
    int prev_size = telemetry_.last_trajectory.x.size();
    if (prev_size > 0) {
      s += prev_size * speed * environment_.time_step_;
    }
    return s;
  }

  bool isObjectAhead(const FusedObject& object, double s) {
    double object_s = predictObjectPosition(object);
    return s < object_s;
  }

  bool isObjectNear(const FusedObject& object, double s) {
    double object_s = predictObjectPosition(object);
    double threshold = ego_.min_distance_to_front_object;
    return abs(s - object_s) < threshold;
  }

  FusedObjects getObjectsInLane(const PredictionData& predictions,
                                std::uint8_t lane_id) {
    FusedObjects result;
    // for (const auto object : predictions.sensor_fusion) {
    for (const auto object : telemetry_.sensor_fusion) {
      if (isObjectInLane(object, lane_id)) {
        result.push_back(object);
      }
    }
    return result;
  }

  FusedObjects getObjectsInFront(const FusedObjects& objects, double car_s) {
    FusedObjects result;
    for (const auto object : objects) {
      if (isObjectAhead(object, car_s)) {
        result.push_back(object);
      }
    }
    return result;
  }

  FusedObjects getObjectsInProximity(const FusedObjects& objects,
                                     double car_s) {
    FusedObjects result;
    for (const auto object : objects) {
      if (isObjectNear(object, car_s)) {
        result.push_back(object);
      }
    }
    return result;
  }

  FusedObject getNearestObject(const FusedObjects& objects, double car_s) {
    FusedObject result;
    double nearest_distance{1000.0};
    for (const auto object : objects) {
      double distance = predictObjectPosition(object);
      if (distance < nearest_distance) {
        result = object;
        nearest_distance = distance;
      }
    }
    return result;
  }

  double getSpeedForecast(const PredictionData& predictions,
                          std::uint8_t lane_id) {
    // TODO: update speed according to acceleration/braking
    // TODO: speed from telemetry is not reliable. Why?
    bool is_object_ahead = false;
    bool is_object_behind = false;

    double car_s = ego_.s;
    if (telemetry_.last_trajectory.x.size() > 1) {
      car_s = telemetry_.end_path_s;
    }

    double object_speed{0.0};
    auto objects_in_lane = getObjectsInLane(predictions, lane_id);
    auto objects_in_front = getObjectsInFront(objects_in_lane, car_s);
    auto objects_near = getObjectsInProximity(objects_in_front, car_s);
    if (!objects_near.empty()) {
      auto object = getNearestObject(objects_near, car_s);
      object_speed = sqrt(object.vx * object.vx + object.vy * object.vy);
      is_object_ahead = true;
    }

    double delta_speed{0};
    if (is_object_ahead) {
      if (is_object_behind) {
        // cannot slow down agressively!
        delta_speed = -0.5 * speed_brake_delta_mps_;
      } else {
        // slow down to keep distance
        delta_speed = -1.0 * fmin(fabs(ego_.speed - object_speed),
                                  speed_brake_delta_mps_);
      }
    } else {
      // keep max velocity possible
      delta_speed = speed_control_delta_mps_;
    }

    return fmax(0, fmin(ego_.speed + delta_speed, ego_.desired_speed));
  }

  bool isLaneChangePossible(const PredictionData& predictions,
                            std::uint8_t intended_lane_id) {
    double car_s = ego_.s;
    if (telemetry_.last_trajectory.x.size() > 0) {
      car_s = telemetry_.end_path_s;
    }

    double gap_half_length = 20.0;
    bool is_gap_free = true;
    auto objects = getObjectsInLane(predictions, intended_lane_id);
    for (auto object : objects) {
      double position = predictObjectPosition(object);
      double distance = fabs(car_s - position);
      if (distance < gap_half_length) {
        // std::cout << "[Generator]: Cannot switch to lane ("
        //           << (int)intended_lane_id
        //           << ") because of car nearby at distance: " << distance
        //           << std::endl;
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

  Trajectory generateKeepLaneTrajectory(const PredictionData& predictions) {
    std::uint8_t endpoint_lane_id = ego_.lane_id;
    std::uint8_t intended_lane_id = ego_.lane_id;
    double speed = getSpeedForecast(predictions, intended_lane_id);
    return generateTrajectoryFromSpline(intended_lane_id, endpoint_lane_id,
                                        speed);
  }

  Trajectory generatePrepareLaneChangeLeftTrajectory(
      const PredictionData& predictions) {
    // TODO: planner should not attempt this when already on leftmost lane
    std::uint8_t intended_lane_id = fmax(0, ego_.lane_id - 1);
    std::uint8_t endpoint_lane_id = ego_.lane_id;

    double speed_actual = getSpeedForecast(predictions, endpoint_lane_id);
    double speed_next = getSpeedForecast(predictions, intended_lane_id);

    double speed;
    bool is_object_behind = false;
    if (is_object_behind) {
      // keep speed of current lane so as to not collide with car behind
      speed = speed_actual;
    } else {
      // prefer the lowest speed from both lanes
      speed = fmin(speed_actual, speed_next);
    }

    return generateTrajectoryFromSpline(intended_lane_id, endpoint_lane_id,
                                        speed);
  }

  Trajectory generatePrepareLaneChangeRightTrajectory(
      const PredictionData& predictions) {
    std::uint8_t intended_lane_id = fmin(2, ego_.lane_id + 1);
    std::uint8_t endpoint_lane_id = ego_.lane_id;

    double speed_actual = getSpeedForecast(predictions, endpoint_lane_id);
    double speed_next = getSpeedForecast(predictions, intended_lane_id);

    double speed;
    bool is_object_behind = false;
    if (is_object_behind) {
      // keep speed of current lane so as to not collide with car behind
      speed = speed_actual;
    } else {
      // prefer the lowest speed from both lanes
      speed = fmin(speed_actual, speed_next);
    }

    return generateTrajectoryFromSpline(intended_lane_id, endpoint_lane_id,
                                        speed);
  }

  Trajectory generateInvalidTrajectory() {
    Trajectory invalid_trajectory;
    invalid_trajectory.characteristics.is_valid = false;
    return invalid_trajectory;
  }

  Trajectory generateLaneChangeTrajectory(const PredictionData& predictions,
                                          std::uint8_t intended_lane_id,
                                          std::uint8_t endpoint_lane_id) {
    if (!isLaneChangePossible(predictions, intended_lane_id)) {
      return generateInvalidTrajectory();
    }
    double speed = getSpeedForecast(predictions, intended_lane_id);
    return generateTrajectoryFromSpline(intended_lane_id, endpoint_lane_id,
                                        speed);
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

  void setTargetData(const TargetData& target) { target_ = target; }

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
  float look_ahead_distance_{50.0F};

  // speed
  float speed_control_delta_mps_{0.1F};
  float speed_brake_delta_mps_{0.2F};
};  // namespace udacity

}  // namespace udacity

#endif  // TRAJECTORY_GENERATOR_H_

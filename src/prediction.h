#ifndef PREDICTION_H_
#define PREDICTION_H_

#include <cmath>
#include <memory>

#include "data_types.h"
#include "helpers.h"
#include "parameters.h"

namespace udacity {
class Prediction {
 public:
  Prediction(std::shared_ptr<Map> map, const Parameters& parameters)
      : map_(map), parameters_(parameters) {}

  void setTelemetry(const TelemetryPacket& telemetry) {
    telemetry_ = telemetry;
  }

  void step() {
    predictions_.lanes.resize(3);
    // TODO: Drop outdated vehicles and update others.
    predictions_.lanes[0].vehicles.clear();
    predictions_.lanes[1].vehicles.clear();
    predictions_.lanes[2].vehicles.clear();
    predictVehicles();
    computeFieldsForLane(0U);
    computeFieldsForLane(1U);
    computeFieldsForLane(2U);
  }

  PredictionData getPredictions() const { return predictions_; }

 private:
  /**
   * Predict vehicle position when at reference point.
   */
  double predictVehiclePositionAtReference(double s, double speed) const {
    double predicted = s;
    int prev_size =
        fmin(parameters_.previous_path_keep, telemetry_.last_path.size());
    if (prev_size > 1) {
      predicted += prev_size * speed * parameters_.time_step_;
    }
    return predicted;
  }

  /**
   * Predicts frenet path assuming keep lane behavior at constant speed.
   */
  FrenetPath getFrenetPath(double s0, double d0, double speed) const {
    FrenetPath result;
    for (int i = 0; i < parameters_.path_size; ++i) {
      double s = s0 + i * speed * parameters_.time_step_;
      result.emplace_back(s, d0);
    }
    return result;
  }

  double getPredictedEgo() const {
    if (telemetry_.last_path.size() > 1) {
      auto pos =
          fmin(parameters_.previous_path_keep, telemetry_.last_path.size());
      float x1 = telemetry_.last_path[pos - 2].x;
      float y1 = telemetry_.last_path[pos - 2].y;
      float x2 = telemetry_.last_path[pos - 1].x;
      float y2 = telemetry_.last_path[pos - 1].y;
      float yaw = atan2(y2 - y1, x2 - x1);

      auto frenet = getFrenet(x2, y2, yaw, map_);
      float s = frenet.s;
      float d = frenet.d;
      return s;
    }
    return telemetry_.car_s;
  }

  Vehicle predictVehicle(const FusedObject& object) const {
    Vehicle v;
    v.id = object.id;
    v.d = object.d;
    v.s = object.s;
    v.speed = sqrt(object.vx * object.vx + object.vy * object.vy);
    v.lane_id = getLaneIdFromFrenet(v.d, parameters_.lane_width);
    v.distance = fabs(v.s - telemetry_.car_s);
    v.predicted_s = predictVehiclePositionAtReference(v.s, v.speed);
    v.predicted_distance = fabs(v.predicted_s - getPredictedEgo());
    if (v.s > telemetry_.car_s) {
      v.is_ahead = true;
      v.is_near = v.predicted_distance < parameters_.safe_distance_ahead;
    } else {
      v.is_ahead = false;
      v.is_near = v.predicted_distance < parameters_.safe_distance_behind;
    }
    v.is_behind = not v.is_ahead;
    v.frenet_path = getFrenetPath(v.s, v.d, v.speed);
    return v;
  }

  void predictVehicles() {
    for (const auto& object : telemetry_.sensor_fusion) {
      Vehicle v = predictVehicle(object);
      auto& lane = predictions_.lanes[v.lane_id];
      lane.vehicles.push_back(v);
    }
  }

  std::pair<bool, Vehicle> getNearestVehicleAhead(std::uint8_t lane_id) const {
    const auto& lane = predictions_.lanes[lane_id];
    double nearest_distance = map_->max_s;
    Vehicle nearest;
    bool found{false};
    for (const auto& vehicle : lane.vehicles) {
      if (vehicle.is_ahead and vehicle.distance < nearest_distance) {
        nearest_distance = vehicle.distance;
        nearest = vehicle;
        found = true;
      }
    }
    return {found, nearest};
  }

  std::pair<bool, Vehicle> getNearestVehicleBehind(std::uint8_t lane_id) const {
    const auto& lane = predictions_.lanes[lane_id];
    double nearest_distance = map_->max_s;
    Vehicle nearest;
    bool found{false};
    for (const auto& vehicle : lane.vehicles) {
      if (vehicle.is_behind and vehicle.distance < nearest_distance) {
        nearest_distance = vehicle.distance;
        nearest = vehicle;
        found = true;
      }
    }
    return {found, nearest};
  }

  void computeFieldsForLane(std::uint8_t lane_id) {
    auto& lane = predictions_.lanes[lane_id];

    // Vehicle Ahead
    const auto nearest_ahead = getNearestVehicleAhead(lane_id);
    lane.has_vehicle_ahead = nearest_ahead.first;
    lane.vehicle_ahead = nearest_ahead.second;

    // Vehicle Ahead
    const auto nearest_behind = getNearestVehicleBehind(lane_id);
    lane.has_vehicle_behind = nearest_behind.first;
    lane.vehicle_behind = nearest_behind.second;

    // lane speed
    if (lane.has_vehicle_ahead and
        lane.vehicle_ahead.distance <
            parameters_.lane_speed_look_ahead_distance) {
      lane.speed = lane.vehicle_ahead.speed;
    } else {
      lane.speed = parameters_.speed_limit;
    }
  }

 private:
  Parameters parameters_;
  TelemetryPacket telemetry_;
  PredictionData predictions_;
  std::shared_ptr<Map> map_;
};

}  // namespace udacity

#endif  // PREDICTION_H_

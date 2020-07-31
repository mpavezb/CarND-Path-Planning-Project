#ifndef PREDICTION_H_
#define PREDICTION_H_

#include <cmath>
#include <memory>

#include "data_types.h"
#include "helpers.h"

namespace udacity {
class Prediction {
 public:
  Prediction(std::shared_ptr<Map> map, const Parameters& parameters)
      : map_(map), parameters_(parameters) {}

  void setTelemetry(const TelemetryPacket& telemetry) {
    telemetry_ = telemetry;
  }

  void step() {
    predictions_.vehicles = predictVehicles();
    updateLaneSpeeds();
  }

  PredictionData getPredictions() { return predictions_; }

 private:
  // Vehicles getVehiclesInLane(std::uint8_t lane_id) {
  //   Vehicles result;
  //   for (const auto& vehicle : predictions_.vehicles) {
  //     if (lane_id == vehicle.lane_id) {
  //       result.push_back(vehicle);
  //     }
  //   }
  //   return result;
  // }

  std::pair<Vehicle, bool> getNearestVehicleAhead(std::uint8_t lane_id) {
    double nearest_distance = map_->max_s;
    Vehicle nearest;
    bool found{false};
    for (const auto& vehicle : predictions_.vehicles) {
      if (vehicle.is_ahead and vehicle.distance < nearest_distance) {
        nearest_distance = vehicle.distance;
        nearest = vehicle;
        found = true;
      }
    }
    return {nearest, found};
  }

  double getLaneSpeed(std::uint8_t lane_id) {
    const auto nearest = getNearestVehicleAhead(lane_id);
    const auto vehicle = nearest.first;
    const auto found = nearest.second;
    if (found and vehicle.distance < parameters_.vehicle_in_front_threshold) {
      return vehicle.speed;
    }
    return parameters_.speed_limit;
  }

  void updateLaneSpeeds() {
    predictions_.lane_speeds.resize(3);
    predictions_.lane_speeds[0] = getLaneSpeed(0);
    predictions_.lane_speeds[1] = getLaneSpeed(1);
    predictions_.lane_speeds[2] = getLaneSpeed(2);
  }

  /**
   * Predict vehicle position when at reference point.
   */
  double predictVehiclePositionAtReference(double s, double speed) {
    double predicted = s;
    int prev_size = telemetry_.last_path.size();
    if (prev_size > 1) {
      predicted += prev_size * speed * parameters_.time_step_;
    }
    return predicted;
  }

  double getPredictedEgo() {
    if (telemetry_.last_path.size() > 1) {
      return telemetry_.end_path_s;
    }
    return telemetry_.car_s;
  }

  Vehicles predictVehicles() {
    Vehicles result;
    for (const auto& fused_object : telemetry_.sensor_fusion) {
      Vehicle v;
      v.id = fused_object.id;
      v.d = fused_object.d;
      v.s = fused_object.s;
      v.speed = sqrt(fused_object.vx * fused_object.vx +
                     fused_object.vy * fused_object.vy);
      v.lane_id = getLaneIdFromFrenet(v.d, parameters_.lane_width);
      v.distance = fabs(v.s - telemetry_.car_s);
      v.predicted_s = predictVehiclePositionAtReference(v.s, v.speed);
      v.predicted_distance = fabs(v.predicted_s - getPredictedEgo());
      v.is_ahead = v.s > telemetry_.car_s;
      v.is_behind = v.s < telemetry_.car_s;
      v.is_near = false;

      result.push_back(v);
    }
    return result;
  }

  // bool isObjectInLane(const FusedObject& object, std::uint8_t lane_id) {
  //   double lane_width = parameters_.lane_width;
  //   double center_lane_d = lane_width * (0.5 + lane_id);
  //   double left_boundary_d = center_lane_d - lane_width / 2.0;
  //   double right_boundary_d = center_lane_d + lane_width / 2.0;
  //   return left_boundary_d < object.d && object.d < right_boundary_d;
  // }

  // bool isObjectNear(const FusedObject& object, double s) {
  //   return true;
  //   // double object_s = predictObjectPosition(object);
  //   // double threshold = ego_.min_distance_to_front_object;
  //   // return abs(s - object_s) < threshold;
  // }

  // FusedObjects getObjectsInLane(const PredictionData& predictions,
  //                               std::uint8_t lane_id) {
  //   FusedObjects result;
  //   // for (const auto object : predictions.sensor_fusion) {
  //   for (const auto object : telemetry_.sensor_fusion) {
  //     if (isObjectInLane(object, lane_id)) {
  //       result.push_back(object);
  //     }
  //   }
  //   return result;
  // }

  // FusedObjects getObjectsInFront(const FusedObjects& objects, double car_s) {
  //   FusedObjects result;
  //   for (const auto object : objects) {
  //     if (isObjectAhead(object, car_s)) {
  //       result.push_back(object);
  //     }
  //   }
  //   return result;
  // }

  // FusedObjects getObjectsInProximity(const FusedObjects& objects,
  //                                    double car_s) {
  //   FusedObjects result;
  //   for (const auto object : objects) {
  //     if (isObjectNear(object, car_s)) {
  //       result.push_back(object);
  //     }
  //   }
  //   return result;
  // }

  // FusedObject getNearestObject(const Vehicle& objects, double car_s) {
  //   FusedObject result;
  //   double nearest_distance{1000.0};
  //   for (const auto object : objects) {
  //     double distance = predictObjectPosition(object);
  //     if (distance < nearest_distance) {
  //       result = object;
  //       nearest_distance = distance;
  //     }
  //   }
  //   return result;
  // }

 private:
  Parameters parameters_;
  TelemetryPacket telemetry_;
  PredictionData predictions_;
  std::shared_ptr<Map> map_;
};

}  // namespace udacity

#endif  // PREDICTION_H_

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

  void step() { predictions_.vehicles = predictVehicles(); }

  PredictionData getPredictions() { return predictions_; }

 private:
  FusedObjects getObjectsInLane(std::uint8_t lane_id) {
    FusedObjects result;
    for (auto object : telemetry_.sensor_fusion) {
      if (lane_id == getLaneIdFromFrenet(object.d, parameters_.lane_width)) {
        result.push_back(object);
      }
    }
    return result;
  }

  bool isVehicleAhead(const Vehicle& vehicle, double s) {
    return s < vehicle.predicted_s;
  }

  bool isObjectInFront(const FusedObject& object) {
    return object.s > telemetry_.car_s;
  }

  double getDistanceToObject(const FusedObject& object) {
    return fabs(object.s - telemetry_.car_s);
  }

  FusedObject getNearestObjectInFront(const FusedObjects& objects) {
    double nearest_distance = map_->max_s;
    FusedObject nearest;
    for (auto object : objects) {
      double distance = getDistanceToObject(object);
      if (isObjectInFront(object) and distance < nearest_distance) {
        nearest_distance = distance;
        nearest = object;
      }
    }
    return nearest;
  }

  double getObjectSpeed(const FusedObject& object) {
    return sqrt(object.vx * object.vx + object.vy * object.vy);
  }

  double getLaneSpeed(std::uint8_t lane_id) {
    double speed = parameters_.speed_limit;
    auto objects = getObjectsInLane(lane_id);
    if (not objects.empty()) {
      auto object = getNearestObjectInFront(objects);
      if (getDistanceToObject(object) <
          parameters_.vehicle_in_front_threshold) {
        speed = getObjectSpeed(object);
      }
    }
    return speed;
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
      v.predicted_s = predictVehiclePositionAtReference(v.s, v.speed);
      v.predicted_distance = v.predicted_s - getPredictedEgo();
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

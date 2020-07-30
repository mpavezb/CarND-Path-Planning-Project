#ifndef PREDICTION_H_
#define PREDICTION_H_

#include <cmath>
#include <memory>

#include "data_types.h"

namespace udacity {
class Prediction {
 public:
  Prediction(std::shared_ptr<Map> map, const Parameters& parameters)
      : map_(map), parameters_(parameters) {}

  void setTelemetry(const TelemetryPacket& telemetry) {
    telemetry_ = telemetry;
  }

  std::uint8_t getLaneIdFromFrenet(double d) {
    return fmax(fmin(2, floor(d / parameters_.lane_width)), 0);
  }

  FusedObjects getObjectsInLane(std::uint8_t lane_id) {
    FusedObjects result;
    for (auto object : telemetry_.sensor_fusion) {
      if (lane_id == getLaneIdFromFrenet(object.d)) {
        result.push_back(object);
      }
    }
    return result;
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
    // std::cout << "[Prediction] Nearest object in front of lane ("
    //           << (int)getLaneIdFromFrenet(nearest.d)
    //           << ") is at distance: " << nearest_distance << std::endl;
    return nearest;
  }

  double getObjectSpeed(const FusedObject& object) {
    return sqrt(object.vx * object.vx + object.vy * object.vy);
  }

  double getLaneSpeed(std::uint8_t lane_id, double distance_threshold = 50) {
    double speed = parameters_.speed_limit;
    auto objects = getObjectsInLane(lane_id);
    if (not objects.empty()) {
      auto object = getNearestObjectInFront(objects);
      if (getDistanceToObject(object) < distance_threshold) {
        speed = getObjectSpeed(object);
      }
    }
    // std::cout << "[Prediction] Speed for lane (" << (int)lane_id
    //           << ") is: " << speed << std::endl;
    return speed;
  }

  void updateLaneSpeeds() {
    predictions.lane_speeds.resize(3);
    predictions.lane_speeds[0] = getLaneSpeed(0);
    predictions.lane_speeds[1] = getLaneSpeed(1);
    predictions.lane_speeds[2] = getLaneSpeed(2);
  }

  void updateVehicles() {
    // TODO: Do real tracking / prediction here!
    for (const auto& fused_object : telemetry_.sensor_fusion) {
      Vehicle v;
      v.id = fused_object.id;
      v.d = fused_object.d;
      v.s = fused_object.s;
      v.lane_id = getLaneIdFromFrenet(fused_object.d);
      v.speed = sqrt(fused_object.vx * fused_object.vx +
                     fused_object.vy * fused_object.vy);
      predictions.vehicles.push_back(v);
    }
  }

  void step() {
    // Do something!
    updateVehicles();
    updateLaneSpeeds();
  }

  bool isObjectInLane(const FusedObject& object, std::uint8_t lane_id) {
    double lane_width = parameters_.lane_width;
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
    int prev_size = telemetry_.last_path.size();
    if (prev_size > 0) {
      s += prev_size * speed * parameters_.time_step_;
    }
    return s;
  }

  bool isObjectAhead(const FusedObject& object, double s) {
    double object_s = predictObjectPosition(object);
    return s < object_s;
  }

  bool isObjectNear(const FusedObject& object, double s) {
    return true;
    // double object_s = predictObjectPosition(object);
    // double threshold = ego_.min_distance_to_front_object;
    // return abs(s - object_s) < threshold;
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

  PredictionData getPredictions() { return predictions; }

 private:
  Parameters parameters_;
  TelemetryPacket telemetry_;
  PredictionData predictions;
  std::shared_ptr<Map> map_;
};

}  // namespace udacity

#endif  // PREDICTION_H_

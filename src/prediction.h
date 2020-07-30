#ifndef PREDICTION_H_
#define PREDICTION_H_

#include <cmath>
#include <memory>

#include "data_types.h"

namespace udacity {
class Prediction {
 public:
  Prediction(const Map& map) { map_ = std::shared_ptr<Map>(new Map(map)); }

  void setTelemetry(const TelemetryPacket& telemetry) {
    telemetry_ = telemetry;
  }

  std::uint8_t getLaneIdFromFrenet(double d) {
    return fmax(fmin(2, floor(d / environment_.lane_width)), 0);
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
    double speed = environment_.speed_limit;
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

  PredictionData getPredictions() { return predictions; }

 private:
  TelemetryPacket telemetry_;
  PredictionData predictions;
  EnvironmentData environment_;
  std::shared_ptr<Map> map_;
};

}  // namespace udacity

#endif  // PREDICTION_H_

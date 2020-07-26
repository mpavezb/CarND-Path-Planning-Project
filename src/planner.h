#ifndef PLANNER_H
#define PLANNER_H

#include <vector>

#include "helpers.h"
#include "json.hpp"

namespace udacity {

struct TelemetryPacket {
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
};

class TelemetryParser {
 public:
  TelemetryPacket fromJson(const nlohmann::json& data) const {
    TelemetryPacket packet;
    // Main car's localization Data
    packet.car_x = data["x"];
    packet.car_y = data["y"];
    packet.car_s = data["s"];
    packet.car_d = data["d"];
    packet.car_yaw = data["yaw"];
    packet.car_speed = data["speed"];
    return packet;
  }
};

class MotionPlanner {
 public:
  void generate_trajectory(const TelemetryPacket& telemetry) {
    /**
     * TODO: define a path made up of (x,y) points that the car will visit
     *   sequentially every .02 seconds
     */
    next_x_vals.clear();
    next_y_vals.clear();
    double dist_inc = 0.5;
    for (int i = 0; i < 50; ++i) {
      next_x_vals.push_back(telemetry.car_x +
                            (dist_inc * i) * cos(deg2rad(telemetry.car_yaw)));
      next_y_vals.push_back(telemetry.car_y +
                            (dist_inc * i) * sin(deg2rad(telemetry.car_yaw)));
    }
  }

  std::vector<double> getNextXVals() const { return next_x_vals; }
  std::vector<double> getNextYVals() const { return next_y_vals; }

 private:
  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;
};
}  // namespace udacity

#endif  // PLANNER_H

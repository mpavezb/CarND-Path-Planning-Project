#ifndef PLANNER_H
#define PLANNER_H

#include <fstream>
#include <vector>

#include "helpers.h"
#include "json.hpp"

namespace udacity {

struct Map {
  double max_s{0.0};
  std::string filename;
  std::vector<double> waypoints_x;
  std::vector<double> waypoints_y;
  std::vector<double> waypoints_s;
  std::vector<double> waypoints_dx;
  std::vector<double> waypoints_dy;
};

class MapReader {
 public:
  static Map readMap(const std::string& filename) {
    Map map{};
    std::ifstream in_map_(filename.c_str(), std::ifstream::in);
    string line;
    while (getline(in_map_, line)) {
      std::istringstream iss(line);
      double x;
      double y;
      float s;
      float d_x;
      float d_y;
      iss >> x;
      iss >> y;
      iss >> s;
      iss >> d_x;
      iss >> d_y;
      map.waypoints_x.push_back(x);
      map.waypoints_y.push_back(y);
      map.waypoints_s.push_back(s);
      map.waypoints_dx.push_back(d_x);
      map.waypoints_dy.push_back(d_y);
    }
    map.filename = filename;
    return map;
  }
};

struct TelemetryPacket {
  // Main car's localization Data
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;

  // A list of all other cars on the same side the road.
  // auto sensor_fusion;

  // Previous path data given to the Planner
  // auto previous_path_x;
  // auto previous_path_y;

  // Previous path's end s and d values
  double end_path_s;
  double end_path_d;
};

class TelemetryParser {
 public:
  static TelemetryPacket fromJson(const nlohmann::json& data) {
    TelemetryPacket packet;
    packet.car_x = data["x"];
    packet.car_y = data["y"];
    packet.car_s = data["s"];
    packet.car_d = data["d"];
    packet.car_yaw = data["yaw"];
    packet.car_speed = data["speed"];
    // previous_path_x = data["previous_path_x"];
    // previous_path_y = data["previous_path_y"];
    packet.end_path_s = data["end_path_s"];
    packet.end_path_d = data["end_path_d"];
    // auto sensor_fusion = data["sensor_fusion"];

    return packet;
  }
};

class MotionPlanner {
 public:
  MotionPlanner(const Map& map) : map_(map) {}

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
  Map map_;
};

}  // namespace udacity

#endif  // PLANNER_H

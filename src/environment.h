#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "helpers.h"
#include "json.hpp"

namespace udacity {

typedef std::vector<double> Path;
typedef std::vector<Path> Paths;

void printPath(const Path& path, const std::string& name) {
  std::stringstream ss;
  ss << name << "[";
  for (int i = 0; i < path.size() - 1; ++i) {
    ss << path[i] << ", ";
  }
  if (path.size() > 0) {
    ss << path[path.size() - 1];
  }
  ss << "]";
  std::cout << ss.str() << std::endl;
}

struct Point {
  double x;
  double y;
};

struct Pose {
  double x;
  double y;
  double yaw;
};

struct SplineAnchors {
  Path x;
  Path y;
  Pose reference;
};

struct Trajectory {
  Path x;
  Path y;
};

struct Map {
  double max_s{0.0};
  std::string filename;
  Path waypoints_x;
  Path waypoints_y;
  Path waypoints_s;
  Path waypoints_dx;
  Path waypoints_dy;
};

class MapReader {
 public:
  static Map readMap(const std::string& filename) {
    Map map{};
    std::ifstream in_map_(filename.c_str(), std::ifstream::in);
    std::string line;
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
  Paths sensor_fusion;

  // Unvisited points from previous trajectory
  Trajectory last_trajectory;

  // Previous path's end s and d values
  double end_path_s;
  double end_path_d;
};

class TelemetryParser {
 public:
  template <typename T>
  static std::vector<T> fromJsonVector(const nlohmann::json& data) {
    std::vector<T> result;
    for (auto value : data) {
      result.push_back(value);
    }
    return result;
  }

  static TelemetryPacket fromJson(const nlohmann::json& data) {
    TelemetryPacket packet;
    packet.car_x = data["x"];
    packet.car_y = data["y"];
    packet.car_s = data["s"];
    packet.car_d = data["d"];
    packet.car_yaw = deg2rad(data["yaw"]);
    packet.car_speed = data["speed"];
    packet.last_trajectory.x = fromJsonVector<double>(data["previous_path_x"]);
    packet.last_trajectory.y = fromJsonVector<double>(data["previous_path_y"]);
    packet.sensor_fusion = fromJsonVector<Path>(data["sensor_fusion"]);
    packet.end_path_s = data["end_path_s"];
    packet.end_path_d = data["end_path_d"];
    return packet;
  }
};

}  // namespace udacity

#endif  // ENVIRONMENT_H

#ifndef SERIALIZATION_H
#define SERIALIZATION_H

#include <fstream>
#include <string>
#include <vector>

#include "data_types.h"
#include "helpers.h"
#include "third_party/json.hpp"

namespace udacity {

class MapReader {
 public:
  static Map readMap(const Parameters& parameters) {
    Map map{};
    std::ifstream in_map_(parameters.map_filename.c_str(), std::ifstream::in);
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
    map.max_s = parameters.max_s;
    return map;
  }
};

void from_json(const nlohmann::json& j, FusedObject& object) {
  const std::vector<double> values = j;
  object.id = values[0];
  object.x = values[1];
  object.y = values[2];
  object.vx = values[3];
  object.vy = values[4];
  object.s = values[5];
  object.d = values[6];
}

void from_json(const nlohmann::json& j, Path& path) {
  std::vector<double> values = j;
  path = values;
}

void from_json(const nlohmann::json& j, TelemetryPacket& packet) {
  packet.car_x = j["x"];
  packet.car_y = j["y"];
  packet.car_s = j["s"];
  packet.car_d = j["d"];
  packet.car_yaw = deg2rad(j["yaw"]);
  packet.car_speed = ((double)j["speed"]) / 2.237;
  packet.last_trajectory_x = j["previous_path_x"].get<Path>();
  packet.last_trajectory_y = j["previous_path_y"].get<Path>();
  packet.sensor_fusion = j["sensor_fusion"].get<FusedObjects>();
  packet.end_path_s = j["end_path_s"];
  packet.end_path_d = j["end_path_d"];
}

}  // namespace udacity

#endif  // SERIALIZATION_H

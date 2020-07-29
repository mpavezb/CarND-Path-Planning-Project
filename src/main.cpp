#include <uWS/uWS.h>

#include <iostream>
#include <string>

//#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"
#include "data_types.h"
#include "helpers.h"
#include "motion_planning.h"
#include "prediction.h"
#include "serialization.h"
#include "third_party/json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using namespace udacity;

int main() {
  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  Map map = MapReader::readMap(map_file_);
  map.max_s = max_s;
  MotionPlanning motion_planning{map};
  Prediction prediction{map};

  uWS::Hub h;
  h.onMessage([&motion_planning, &prediction](uWS::WebSocket<uWS::SERVER> ws,
                                              char *data, size_t length,
                                              uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message
    // event. The 4 signifies a websocket message The 2 signifies a
    // websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          TelemetryPacket telemetry = j[1];

          prediction.setTelemetry(telemetry);
          prediction.step();

          motion_planning.setTelemetry(telemetry);
          motion_planning.setPredictions(prediction.getPredictions());
          motion_planning.step();

          Trajectory trajectory = motion_planning.getTrajectory();

          json msgJson;
          msgJson["next_x"] = trajectory.x;
          msgJson["next_y"] = trajectory.y;
          auto msg = "42[\"control\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  });  // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}

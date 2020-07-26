#ifndef PLANNER_H
#define PLANNER_H

#include <vector>

#include "environment.h"
#include "helpers.h"

namespace udacity {

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

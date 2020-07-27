#ifndef MOTION_PLANNING_H_
#define MOTION_PLANNING_H_

#include <iostream>
#include <memory>
#include <vector>

#include "data_types.h"
#include "state_machine.h"
#include "trajectory_generator.h"

namespace udacity {

class MotionPlanning {
 public:
  MotionPlanning(const Map& map) {
    map_ = std::shared_ptr<Map>(new Map(map));
    generator_ =
        std::unique_ptr<TrajectoryGenerator>(new TrajectoryGenerator());
    generator_->setMap(map_);

    StateMachine::StateMachine::start();
  }

  void setTelemetry(const TelemetryPacket& telemetry) {
    telemetry_ = telemetry;
  }

  void step() {
    UpdateEvent event;
    StateMachine::dispatch(event);
  }

  Trajectory getTrajectory() {
    return generator_->generateTrajectory(telemetry_);
  }

 private:
  TelemetryPacket telemetry_;
  std::shared_ptr<Map> map_;
  std::unique_ptr<TrajectoryGenerator> generator_;
};

}  // namespace udacity

#endif  // MOTION_PLANNING_H_

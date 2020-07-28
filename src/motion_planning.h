#ifndef MOTION_PLANNING_H_
#define MOTION_PLANNING_H_

#include <iostream>
#include <memory>
#include <vector>

#include "data_types.h"
#include "state_machine.h"
#include "trajectory_generator.h"
#include "trajectory_validator.h"

namespace udacity {

class MotionPlanning {
 public:
  MotionPlanning(const Map& map) {
    map_ = std::shared_ptr<Map>(new Map(map));
    generator_ =
        std::shared_ptr<TrajectoryGenerator>(new TrajectoryGenerator());
    validator_ =
        std::shared_ptr<TrajectoryValidator>(new TrajectoryValidator());
    generator_->setMap(map_);

    // Not good, but seems like the only way to share data with the tiny FSM
    // is through the payload.
    sm_event_.functions.generator = generator_;
    sm_event_.functions.validator = validator_;
    sm_event_.output =
        std::shared_ptr<UpdateEvent::Output>(new UpdateEvent::Output());

    StateMachine::StateMachine::start();
  }

  void updateEgoLaneId(const TelemetryPacket& telemetry) {
    ego_.lane_id =
        fmax(fmin(2, floor(telemetry.car_d / environment_.lane_width)), 0);
  }

  void setTelemetry(const TelemetryPacket& telemetry) {
    generator_->setTelemetry(telemetry);
    updateEgoLaneId(telemetry);
  }

  void setPredictions(const PredictionData& predictions) {
    sm_event_.input.predictions = predictions;
    validator_->setPredictionData(predictions);
  }

  void step() {
    generator_->setEgoStatus(ego_);
    validator_->setEgoStatus(ego_);
    StateMachine::dispatch(sm_event_);
    selected_trajectory_ = sm_event_.output->selected_trajectory;
    ego_.speed = selected_trajectory_.characteristics.speed;
  }

  Trajectory getTrajectory() { return selected_trajectory_; }

 private:
  EnvironmentData environment_;
  EgoStatus ego_;
  UpdateEvent sm_event_;
  Trajectory selected_trajectory_;
  std::shared_ptr<Map> map_;
  std::shared_ptr<TrajectoryGenerator> generator_;
  std::shared_ptr<TrajectoryValidator> validator_;
};

}  // namespace udacity

#endif  // MOTION_PLANNING_H_

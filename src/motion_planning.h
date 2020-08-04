#ifndef MOTION_PLANNING_H_
#define MOTION_PLANNING_H_

#include <memory>
#include <vector>

#include "data_types.h"
#include "helpers.h"
#include "state_machine.h"
#include "trajectory_generator.h"
#include "trajectory_validator.h"

namespace udacity {

class MotionPlanning {
 public:
  MotionPlanning(std::shared_ptr<Map> map, const Parameters& parameters)
      : map_(map),
        parameters_(parameters),
        generator_{new TrajectoryGenerator(map, parameters)},
        validator_{new TrajectoryValidator(parameters)} {
    // Not good, but seems like the only way to share data with the tiny FSM
    // is through the payload.
    sm_event_.functions.generator = generator_;
    sm_event_.functions.validator = validator_;
    sm_event_.output =
        std::shared_ptr<UpdateEvent::Output>(new UpdateEvent::Output());

    StateMachine::StateMachine::start();
  }

  void setTelemetry(const TelemetryPacket& telemetry) {
    generator_->setTelemetry(telemetry);
    // using s, d from telemetry is not that good!
    ego_.s = telemetry.car_s;
    ego_.lane_id = getLaneIdFromFrenet(telemetry.car_d, parameters_.lane_width);
    // std::cout << "Car is on lane_id: " << static_cast<int>(ego_.lane_id)
    //           << " (target id is: "
    //           << static_cast<int>(parameters_.goal_lane_id) << ")" <<
    //           std::endl;
  }

  void setPredictions(const PredictionData& predictions) {
    generator_->setPredictions(predictions);
    validator_->setPredictionData(predictions);
  }

  void step() {
    generator_->setEgoStatus(ego_);
    validator_->setEgoStatus(ego_);
    generator_->step();
    StateMachine::dispatch(sm_event_);
    selected_trajectory_ = sm_event_.output->selected_trajectory;
    ego_.speed = selected_trajectory_.characteristics.speed;
    std::cout << "State is: " << selected_trajectory_.characteristics.action
              << std::endl;
  }

  Trajectory getTrajectory() { return selected_trajectory_; }

 private:
  Parameters parameters_;
  EgoStatus ego_;
  UpdateEvent sm_event_;
  Trajectory selected_trajectory_;
  std::shared_ptr<Map> map_;
  std::shared_ptr<TrajectoryGenerator> generator_;
  std::shared_ptr<TrajectoryValidator> validator_;
};

}  // namespace udacity

#endif  // MOTION_PLANNING_H_

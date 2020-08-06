#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#include <iostream>
#include <set>
#include <vector>

#include "data_types.h"
#include "third_party/tinyfsm.hpp"
#include "trajectory_generator.h"
#include "trajectory_validator.h"

namespace udacity {

// tinyfsm Event
struct UpdateEvent : public tinyfsm::Event {
  struct Functions {
    std::shared_ptr<TrajectoryGenerator> generator;
    std::shared_ptr<TrajectoryValidator> validator;
  };
  struct Output {
    Trajectory selected_trajectory;
  };
  Functions functions;
  std::shared_ptr<Output> output;
};

// tinyfsm States
class KeepLaneState;
class PrepareLaneChangeLeftState;
class PrepareLaneChangeRightState;
class LaneChangeLeftState;
class LaneChangeRightState;

// tinyfsm State Machine
class StateMachine : public tinyfsm::Fsm<StateMachine> {
 public:
  void react(tinyfsm::Event const &) {}

  virtual std::vector<TrajectoryAction> getValidActions() = 0;
  virtual TrajectoryAction getAction() = 0;

  bool isActionActive(TrajectoryAction action) { return action == getAction(); }

  void updateDebounceTimer() {
    debounce_timer_ = fmin(debounce_limit_steps_, debounce_timer_ + 1);
  }

  bool isDebounceTimeout() { return debounce_timer_ == debounce_limit_steps_; }

  void executeTransition(TrajectoryAction action) {
    if (isActionActive(action)) return;
    switch (action) {
      case TrajectoryAction::kKeepLane:
        transit<KeepLaneState>([] {});
        break;
      case TrajectoryAction::kPrepareChangeLaneLeft:
        transit<PrepareLaneChangeLeftState>([] {});
        break;
      case TrajectoryAction::kPrepareChangeLaneRight:
        transit<PrepareLaneChangeRightState>([] {});
        break;
      case TrajectoryAction::kChangeLaneLeft:
        transit<LaneChangeLeftState>([] {});
        break;
      case TrajectoryAction::kChangeLaneRight:
        transit<LaneChangeRightState>([] {});
        break;
    }
  }

  void react(UpdateEvent const &event) {
    auto generator = event.functions.generator;
    auto validator = event.functions.validator;

    std::vector<Trajectory> candidates;
    for (const auto &action : getValidActions()) {
      auto trajectory = generator->getTrajectoryForAction(action);
      candidates.push_back(trajectory);
    }

    std::set<Trajectory> valid_trajectories;
    for (auto &trajectory : candidates) {
      if (validator->isTrajectoryValid(trajectory)) {
        trajectory.characteristics.cost =
            validator->getTrajectoryCost(trajectory);
        valid_trajectories.insert(trajectory);
        std::cout << "Trajectory '" << trajectory.characteristics.action
                  << "' has cost: " << trajectory.characteristics.cost
                  << std::endl;
      } else {
        std::cout << "Trajectory '" << trajectory.characteristics.action
                  << "' is not valid" << std::endl;
      }
    }
    if (valid_trajectories.empty()) {
      std::cerr
          << "[StateMachine]: There are no valid trajectories! Skipping cycle"
          << std::endl;
      return;
    }
    auto best_trajectory = *valid_trajectories.begin();
    auto best_action = best_trajectory.characteristics.action;

    // only apply debounce to KeepLane->PLCX
    if (getAction() == TrajectoryAction::kKeepLane) {
      updateDebounceTimer();
      if (not isDebounceTimeout()) {
        event.output->selected_trajectory =
            generator->getTrajectoryForAction(TrajectoryAction::kKeepLane);
        return;
      }
    }
    event.output->selected_trajectory = best_trajectory;
    executeTransition(best_action);
  };
  virtual void entry(void) { debounce_timer_ = 0; }
  virtual void exit(void) {}

 private:
  const std::uint8_t debounce_limit_steps_{10U};
  std::uint8_t debounce_timer_{0};
};

class KeepLaneState : public StateMachine {
  TrajectoryAction getAction() override { return TrajectoryAction::kKeepLane; }
  std::vector<TrajectoryAction> getValidActions() override {
    return {TrajectoryAction::kKeepLane,
            TrajectoryAction::kPrepareChangeLaneLeft,
            TrajectoryAction::kPrepareChangeLaneRight};
  }
};
class PrepareLaneChangeLeftState : public StateMachine {
  TrajectoryAction getAction() override {
    return TrajectoryAction::kPrepareChangeLaneLeft;
  }
  std::vector<TrajectoryAction> getValidActions() override {
    return {TrajectoryAction::kKeepLane,
            TrajectoryAction::kPrepareChangeLaneLeft,
            TrajectoryAction::kChangeLaneLeft};
  }
};
class PrepareLaneChangeRightState : public StateMachine {
  TrajectoryAction getAction() override {
    return TrajectoryAction::kPrepareChangeLaneRight;
  }
  std::vector<TrajectoryAction> getValidActions() override {
    return {TrajectoryAction::kKeepLane,
            TrajectoryAction::kPrepareChangeLaneRight,
            TrajectoryAction::kChangeLaneRight};
  }
};
class LaneChangeLeftState : public StateMachine {
  TrajectoryAction getAction() override {
    return TrajectoryAction::kChangeLaneLeft;
  }
  std::vector<TrajectoryAction> getValidActions() override {
    return {TrajectoryAction::kKeepLane, TrajectoryAction::kChangeLaneLeft};
  }
};
class LaneChangeRightState : public StateMachine {
  TrajectoryAction getAction() override {
    return TrajectoryAction::kChangeLaneRight;
  }
  std::vector<TrajectoryAction> getValidActions() override {
    return {TrajectoryAction::kKeepLane, TrajectoryAction::kChangeLaneRight};
  }
};

}  // namespace udacity

FSM_INITIAL_STATE(udacity::StateMachine, udacity::KeepLaneState);

#endif  // STATE_MACHINE_H_

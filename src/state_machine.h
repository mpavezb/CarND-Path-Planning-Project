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
  virtual bool isActionActive(TrajectoryAction) = 0;

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
    event.output->selected_trajectory = best_trajectory;
    executeTransition(best_action);
  };
  virtual void entry(void) {}
  virtual void exit(void) {}
};

class KeepLaneState : public StateMachine {
  bool isActionActive(TrajectoryAction action) override {
    return action == TrajectoryAction::kKeepLane;
  }
  std::vector<TrajectoryAction> getValidActions() override {
    return {TrajectoryAction::kKeepLane,
            TrajectoryAction::kPrepareChangeLaneLeft,
            TrajectoryAction::kPrepareChangeLaneRight};
  }
  void entry(void) override {
    std::cout << "[StateMachine]: Switched to: KeepLaneState." << std::endl;
  }
};
class PrepareLaneChangeLeftState : public StateMachine {
  bool isActionActive(TrajectoryAction action) override {
    return action == TrajectoryAction::kPrepareChangeLaneLeft;
  }
  std::vector<TrajectoryAction> getValidActions() override {
    return {TrajectoryAction::kKeepLane,
            TrajectoryAction::kPrepareChangeLaneLeft,
            TrajectoryAction::kChangeLaneLeft};
  }
  void entry(void) override {
    std::cout << "[StateMachine]: Switched to: PrepareLaneChangeLeftState."
              << std::endl;
  }
};
class PrepareLaneChangeRightState : public StateMachine {
  bool isActionActive(TrajectoryAction action) override {
    return action == TrajectoryAction::kPrepareChangeLaneRight;
  }
  std::vector<TrajectoryAction> getValidActions() override {
    return {TrajectoryAction::kKeepLane,
            TrajectoryAction::kPrepareChangeLaneRight,
            TrajectoryAction::kChangeLaneRight};
  }
  void entry(void) override {
    std::cout << "[StateMachine]: Switched to: PrepareLaneChangeRightState."
              << std::endl;
  }
};
class LaneChangeLeftState : public StateMachine {
  bool isActionActive(TrajectoryAction action) override {
    return action == TrajectoryAction::kChangeLaneLeft;
  }
  std::vector<TrajectoryAction> getValidActions() override {
    return {TrajectoryAction::kKeepLane, TrajectoryAction::kChangeLaneLeft};
  }
  void entry(void) override {
    std::cout << "[StateMachine]: Switched to: LaneChangeLeftState."
              << std::endl;
  }
};
class LaneChangeRightState : public StateMachine {
  bool isActionActive(TrajectoryAction action) override {
    return action == TrajectoryAction::kChangeLaneRight;
  }
  std::vector<TrajectoryAction> getValidActions() override {
    return {TrajectoryAction::kKeepLane, TrajectoryAction::kChangeLaneRight};
  }
  void entry(void) override {
    std::cout << "[StateMachine]: Switched to: LaneChangeRightState."
              << std::endl;
  }
};

}  // namespace udacity

FSM_INITIAL_STATE(udacity::StateMachine, udacity::KeepLaneState);

#endif  // STATE_MACHINE_H_

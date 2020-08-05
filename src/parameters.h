#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <string>

namespace udacity {

struct Parameters {
  // map
  std::string map_filename{"../data/highway_map.csv"};
  float max_s = 6945.554F;
  float lane_width{4.0F};
  float speed_limit{50.0 / 2.237F};

  // lifecycle
  float time_step_{0.02F};

  // target
  std::uint8_t goal_lane_id{1U};  // 0=left, 1=middle, 2=right
  float goal_s{max_s};

  // safety
  float safe_distance{30.0F};

  // control
  float desired_speed{49.5F / 2.237F};
  float lane_look_ahead_distance{50.0F};

  // trajectory generation
  int path_size_{50};
  int n_anchors_{5};
  float anchors_look_ahead_distance{90.0F};
  float trajectory_length{30.0F};
  float acceleration{0.1F};
  float deceleration{0.1F};
};

}  // namespace udacity

#endif  // PARAMETERS_H

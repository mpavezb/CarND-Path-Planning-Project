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

  // speed control
  float safe_distance_ahead{20.0F};
  float safe_distance_behind{10.0F};
  float keep_distance_delta_speed{0.2};
  float desired_speed{49.5F / 2.237F};
  float max_acceleration{0.1F};
  float max_deceleration{0.2F};

  float lane_speed_look_ahead_distance{50.0F};

  // trajectory generation
  int path_size{50};
  int previous_path_keep{10};
  int n_anchors{5};
  float anchors_look_ahead_distance{90.0F};
  float trajectory_length{30.0F};

  // cost
  double cost_empty_lane_dmax{100};
  double cost_empty_lane_cmax{0.5};

  // collision detection
  double collision_th_s{7};
  double collision_th_d{1.5};
  double collision_steps{30};
};

}  // namespace udacity

#endif  // PARAMETERS_H

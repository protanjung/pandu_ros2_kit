#ifndef PURE_PURSUIT_HPP_
#define PURE_PURSUIT_HPP_

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"

class PurePursuit {
 private:
  bool _is_initialized = false;

  float *_x;
  float *_y;
  float *_theta;
  std::vector<geometry_msgs::msg::Point> *_path;

 public:
  float wheel_base;
  float look_ahead_distance;
  unsigned int goal_index_start;
  unsigned int goal_index_stop;
  float goal_x;
  float goal_y;
  float icr_x;
  float icr_y;
  float icr_radius;
  float steering_angle;

  PurePursuit();

  bool init(
      float *x,
      float *y,
      float *theta,
      std::vector<geometry_msgs::msg::Point> *path,
      float _wheel_base,
      float _look_ahead_distance);
  bool is_initialized();

  void set_x_y_theta(float *x, float *y, float *theta);
  void set_path(std::vector<geometry_msgs::msg::Point> *path);

  void update_all();
  void update_goal();
  void update_icr();
  void update_steering();
};

#endif  // PURE_PURSUIT_HPP_
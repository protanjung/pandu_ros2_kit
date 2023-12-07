#include "pandu_ros2_kit/pure_pursuit.hpp"

PurePursuit::PurePursuit() {}

//======================================

bool PurePursuit::init(
    float *x,
    float *y,
    float *theta,
    std::vector<geometry_msgs::msg::Point> *path,
    float _wheel_base,
    float _look_ahead_distance) {
  if (_is_initialized) { return true; }

  _x = x;
  _y = y;
  _theta = theta;
  _path = path;
  wheel_base = _wheel_base;
  look_ahead_distance = _look_ahead_distance;

  _is_initialized = true;
  return true;
}

bool PurePursuit::is_initialized() { return _is_initialized; }

//======================================

void PurePursuit::set_x_y_theta(float *x, float *y, float *theta) {
  _x = x;
  _y = y;
  _theta = theta;
}

void PurePursuit::set_path(std::vector<geometry_msgs::msg::Point> *path) { _path = path; }

//======================================

void PurePursuit::update_all() {
  if (!_is_initialized || _path->size() < 2) {
    goal_index_start = 0;
    goal_index_stop = 0;
    goal_x = *_x;
    goal_y = *_y;
    icr_radius = 0;
    icr_x = *_x;
    icr_y = *_y;
    steering_angle = 0;
    return;
  }

  update_goal();
  update_icr();
  update_steering();
}

void PurePursuit::update_goal() {
  static uint8_t status_normal = 0;
  static uint8_t index_normal = 1;

  if (index_normal >= _path->size()) {
    status_normal = 0;
    index_normal = 1;
  }

  status_normal = 0;

  if (status_normal == 0) {
    for (size_t i = index_normal; i < _path->size(); i++) {
      float distance_backward = sqrtf(powf(_path->at(i - 1).x - *_x, 2) + powf(_path->at(i - 1).y - *_y, 2));
      float distance_forward = sqrtf(powf(_path->at(i - 0).x - *_x, 2) + powf(_path->at(i - 0).y - *_y, 2));
      if (distance_backward < look_ahead_distance && distance_forward > look_ahead_distance) {
        status_normal = 1;
        index_normal = i;
        break;
      }
    }
  }

  if (status_normal == 0) {
    for (size_t i = 1; i < index_normal; i++) {
      float distance_backward = sqrtf(powf(_path->at(i - 1).x - *_x, 2) + powf(_path->at(i - 1).y - *_y, 2));
      float distance_forward = sqrtf(powf(_path->at(i - 0).x - *_x, 2) + powf(_path->at(i - 0).y - *_y, 2));
      if (distance_backward < look_ahead_distance && distance_forward > look_ahead_distance) {
        status_normal = 1;
        index_normal = i;
        break;
      }
    }
  }

  if (status_normal == 0) {
    float nearest_distance = __FLT_MAX__;
    for (size_t i = 1; i < _path->size(); i++) {
      float distance = sqrtf(powf(_path->at(i - 0).x - *_x, 2) + powf(_path->at(i - 0).y - *_y, 2));
      if (distance < nearest_distance) {
        nearest_distance = distance;
        status_normal = 1;
        index_normal = i;
      }
    }
  }

  goal_index_start = index_normal - 1;
  goal_index_stop = index_normal - 0;

  float x1_minus_x0 = _path->at(goal_index_stop).x - _path->at(goal_index_start).x;
  float y1_minus_y0 = _path->at(goal_index_stop).y - _path->at(goal_index_start).y;
  float x0_minus_x = _path->at(goal_index_start).x - *_x;
  float y0_minus_y = _path->at(goal_index_start).y - *_y;

  float a = x1_minus_x0 * x1_minus_x0 + y1_minus_y0 * y1_minus_y0;
  float b = 2 * (x1_minus_x0 * x0_minus_x + y1_minus_y0 * y0_minus_y);
  float c = x0_minus_x * x0_minus_x + y0_minus_y * y0_minus_y - look_ahead_distance * look_ahead_distance;
  float d = b * b - 4 * a * c;

  if (d < 0) {
    goal_x = _path->at(goal_index_stop).x;
    goal_y = _path->at(goal_index_stop).y;
  } else {
    float t1 = (-b + sqrtf(d)) / (2 * a);
    float t2 = (-b - sqrtf(d)) / (2 * a);

    if (t1 >= 0 && t1 <= 1) {
      goal_x = _path->at(goal_index_start).x + t1 * x1_minus_x0;
      goal_y = _path->at(goal_index_start).y + t1 * y1_minus_y0;
    } else if (t2 >= 0 && t2 <= 1) {
      goal_x = _path->at(goal_index_start).x + t2 * x1_minus_x0;
      goal_y = _path->at(goal_index_start).y + t2 * y1_minus_y0;
    } else {
      goal_x = _path->at(goal_index_stop).x;
      goal_y = _path->at(goal_index_stop).y;
    }
  }
}

void PurePursuit::update_icr() {
  float a = atan2(goal_y - *_y, goal_x - *_x) - *_theta;
  icr_radius = look_ahead_distance / (2 * sinf(a));
  icr_x = *_x + icr_radius * cosf(*_theta + M_PI_2);
  icr_y = *_y + icr_radius * sinf(*_theta + M_PI_2);
}

void PurePursuit::update_steering() {
  float a = atan2(goal_y - *_y, goal_x - *_x) - *_theta;
  steering_angle = atan2(2 * wheel_base * sinf(a), look_ahead_distance);
}

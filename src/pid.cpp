#include "pandu_ros2_kit/pid.hpp"

PID::PID() {}

//==============================================================================

void PID::init(
    double kp,
    double ki,
    double kd,
    double dt,
    double min_output,
    double max_output,
    double min_integral,
    double max_integral) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
  _dt = dt;
  _min_output = min_output;
  _max_output = max_output;
  _min_integral = min_integral;
  _max_integral = max_integral;
}

//==============================================================================

double PID::update(double error) {
  static rclcpp::Time time_old = rclcpp::Time(0, 0, RCL_ROS_TIME);
  static rclcpp::Time time_now = rclcpp::Clock().now();
  time_old = time_now;
  time_now = rclcpp::Clock().now();

  if (time_now - time_old > 1s) {
    _error_last = 0;
    _integral = 0;
  }

  _error = error;
  _propotional = _kp * _error;
  _derivative = _kd * (_error - _error_last) / _dt;
  _integral += _ki * _error * _dt;

  if (_integral > _max_integral) {
    _integral = _max_integral;
  } else if (_integral < _min_integral) {
    _integral = _min_integral;
  }

  _output = _propotional + _derivative + _integral;

  if (_output > _max_output) {
    _output = _max_output;
  } else if (_output < _min_output) {
    _output = _min_output;
  }

  _error_last = _error;

  return _output;
}

double PID::update(double error, float min_output, float max_output) {
  _min_output = min_output;
  _max_output = max_output;
  _min_integral = min_output;
  _max_integral = max_output;

  return update(error);
}

double PID::update(double setpoint, double input) { return update(setpoint - input); }

double PID::update(double setpoint, double input, float min_output, float max_output) {
  _min_output = min_output;
  _max_output = max_output;
  _min_integral = min_output;
  _max_integral = max_output;

  return update(setpoint, input);
}

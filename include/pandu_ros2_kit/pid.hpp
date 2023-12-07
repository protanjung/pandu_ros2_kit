#ifndef PID_HPP_
#define PID_HPP_

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class PID {
 private:
  double _kp, _ki, _kd, _dt;
  double _error, _error_last;
  double _propotional;
  double _derivative;
  double _integral, _min_integral, _max_integral;
  double _output, _min_output, _max_output;

 public:
  PID();

  void init(
      double kp,
      double ki,
      double kd,
      double dt,
      double min_output = -1,
      double max_output = 1,
      double min_integral = -1,
      double max_integral = 1);

  double update(double error);
  double update(double error, float min_output, float max_output);
  double update(double setpoint, double input);
  double update(double setpoint, double input, float min_output, float max_output);
};

#endif  // PID_HPP_

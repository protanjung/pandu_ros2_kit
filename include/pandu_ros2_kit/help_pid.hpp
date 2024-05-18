/**
 * @file help_pid.hpp
 * @author Pandu Surya Tantra (pandustantra@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-08
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef HELP_PID_HPP_
#define HELP_PID_HPP_

#include "pandu_ros2_kit/help_logger.hpp"
#include "rclcpp/rclcpp.hpp"

class HelpPID {
  private:
    bool _is_initialized = false;

    HelpLogger _logger;

    double _kp, _ki, _kd;
    double _error, _error_old;
    double _propotional;
    double _derivative;
    double _integral, _min_integral, _max_integral;
    double _output, _min_output, _max_output;

  public:
    HelpPID();

    bool is_initialized();

    bool init(float kp, float ki, float kd, float min_integral, float max_integral, float min_output, float max_output);

    float update(float error);
    float update(float error, float min_output, float max_output);
    float update(float setpoint, float feedback);
    float update(float setpoint, float feedback, float min_output, float max_output);
};

#endif // HELP_PID_HPP_
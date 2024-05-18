/**
 * @file help_pid.cpp
 * @author Pandu Surya Tantra (pandustantra@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-08
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "pandu_ros2_kit/help_pid.hpp"

HelpPID::HelpPID() {}

bool
HelpPID::is_initialized() {
    return _is_initialized;
}

bool
HelpPID::init(float kp, float ki, float kd, float min_integral, float max_integral, float min_output,
              float max_output) {
    if (_is_initialized) {
        return true;
    }

    if (!_logger.init()) {
        return false;
    }

    _kp = kp;
    _ki = ki;
    _kd = kd;
    _min_integral = min_integral;
    _max_integral = max_integral;
    _min_output = min_output;
    _max_output = max_output;

    _is_initialized = true;
    return true;
}

float
HelpPID::update(float error) {
    if (!_is_initialized) {
        return 0;
    }

    static rclcpp::Time time_old = rclcpp::Time(0, 0);
    static rclcpp::Time time_now = rclcpp::Clock().now();
    time_old = time_now;
    time_now = rclcpp::Clock().now();
    double dt = (time_now - time_old).seconds();

    if (dt > 1.0) {
        _error_old = 0;
        _integral = 0;
        _output = 0;
    } else {
        _error = error;
        _propotional = _kp * _error;
        _integral = fminf(fmaxf(_integral + _ki * _error * dt, _min_integral), _max_integral);
        _derivative = _kd * (_error - _error_old) / dt;
        _output = fminf(fmaxf(_propotional + _integral + _derivative, _min_output), _max_output);
        _error_old = _error;
    }

    return _output;
}

float
HelpPID::update(float error, float min_output, float max_output) {
    if (!_is_initialized) {
        return 0;
    }

    _min_output = min_output;
    _max_output = max_output;
    _min_integral = min_output;
    _max_integral = max_output;

    return update(error);
}

float
HelpPID::update(float setpoint, float feedback) {
    if (!_is_initialized) {
        return 0;
    }

    return update(setpoint - feedback);
}

float
HelpPID::update(float setpoint, float feedback, float min_output, float max_output) {
    if (!_is_initialized) {
        return 0;
    }

    return update(setpoint - feedback, min_output, max_output);
}
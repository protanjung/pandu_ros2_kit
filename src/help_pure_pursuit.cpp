/**
 * @file help_pure_pursuit.cpp
 * @author Pandu Surya Tantra (pandustantra@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-06
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "pandu_ros2_kit/help_pure_pursuit.hpp"

HelpPurePursuit::HelpPurePursuit() {}

bool
HelpPurePursuit::is_initialized() {
    return _is_initialized;
}

bool
HelpPurePursuit::init_as_ackermann(float* x, float* y, float* theta, std::vector<geometry_msgs::msg::Point>* path,
                                   float wheel_base, float lookahead_distance) {
    if (_is_initialized) {
        return true;
    }

    if (!_logger.init()) {
        return false;
    }

    _x = x;
    _y = y;
    _theta = theta;
    _path = path;

    _wheel_base = wheel_base;
    _lookahead_distance = lookahead_distance;

    _is_initialized = _is_ackermann = true;
    return true;
}

bool
HelpPurePursuit::init_as_holonomic(float* x, float* y, float* theta, std::vector<geometry_msgs::msg::Point>* path,
                                   float lookahead_distance) {
    if (_is_initialized) {
        return true;
    }

    if (!_logger.init()) {
        return false;
    }

    _x = x;
    _y = y;
    _theta = theta;
    _path = path;

    _lookahead_distance = lookahead_distance;

    _is_initialized = _is_holonomic = true;
    return true;
}

void
HelpPurePursuit::update() {
    if (!_is_initialized || _path->size() < 2) {
        goal_x = *_x;
        goal_y = *_y;
        icr_radius = 0;
        icr_x = *_x;
        icr_y = *_y;
        return;
    }

    _update_goal();
    _update_icr();
}

void
HelpPurePursuit::set_path(std::vector<geometry_msgs::msg::Point>* path) {
    _path = path;
}

void
HelpPurePursuit::set_lookahead_distance(float lookahead_distance) {
    _lookahead_distance = lookahead_distance;
}

void
HelpPurePursuit::set_wheel_base(float wheel_base) {
    _wheel_base = wheel_base;
}

float
HelpPurePursuit::get_lookahead_distance() {
    return _lookahead_distance;
}

float
HelpPurePursuit::get_wheel_base() {
    return _wheel_base;
}

void
HelpPurePursuit::process_ackermann(float* steering_angle) {
    if (!_is_initialized || !_is_ackermann || _path->size() < 2) {
        *steering_angle = 0;
        return;
    }

    float a = atan2(goal_y - *_y, goal_x - *_x) - *_theta;
    *steering_angle = atan2(2 * _wheel_base * sin(a), _lookahead_distance);
}

void
HelpPurePursuit::process_holonomic(float* lin_vel, float* ang_vel, float max_lin_vel, float max_ang_vel) {
    if (!_is_initialized || !_is_holonomic || _path->size() < 2) {
        *ang_vel = 0;
        *lin_vel = 0;
        return;
    }

    float angle_difference = atan2(goal_y - *_y, goal_x - *_x) - *_theta;
    if (angle_difference > M_PI) {
        angle_difference -= 2 * M_PI;
    } else if (angle_difference < -M_PI) {
        angle_difference += 2 * M_PI;
    }

    bool is_facing_goal = (angle_difference >= -M_PI_2 && angle_difference <= M_PI_2);
    bool is_turning_left = (angle_difference >= 0);

    *lin_vel = max_lin_vel;
    *ang_vel = fmin(max_ang_vel, fmax(-max_ang_vel, *lin_vel / icr_radius));
    *lin_vel = *ang_vel * icr_radius;

    if (!is_facing_goal) {
        *lin_vel = 0;

        if (is_turning_left) {
            *ang_vel = max_ang_vel;
        } else {
            *ang_vel = -max_ang_vel;
        }
    }
}

void
HelpPurePursuit::_update_goal() {
    static bool is_normal = false;
    static uint32_t id_normal = 1;

    if (id_normal >= _path->size()) {
        id_normal = 1;
    }

    is_normal = false;

    if (!is_normal) {
        for (size_t i = id_normal; i < _path->size(); i++) {
            float d_backward = sqrt(pow(_path->at(i - 1).x - *_x, 2) + pow(_path->at(i - 1).y - *_y, 2));
            float d_forward = sqrt(pow(_path->at(i - 0).x - *_x, 2) + pow(_path->at(i - 0).y - *_y, 2));
            if (d_backward < _lookahead_distance && d_forward >= _lookahead_distance) {
                is_normal = true;
                id_normal = i;
            }
        }
    }

    if (!is_normal) {
        for (size_t i = 1; i < id_normal; i++) {
            float d_backward = sqrt(pow(_path->at(i - 1).x - *_x, 2) + pow(_path->at(i - 1).y - *_y, 2));
            float d_forward = sqrt(pow(_path->at(i - 0).x - *_x, 2) + pow(_path->at(i - 0).y - *_y, 2));
            if (d_backward < _lookahead_distance && d_forward >= _lookahead_distance) {
                is_normal = true;
                id_normal = i;
            }
        }
    }

    if (!is_normal) {
        float d_min = __FLT_MAX__;
        for (size_t i = 1; i < _path->size(); i++) {
            float d = sqrt(pow(_path->at(i).x - *_x, 2) + pow(_path->at(i).y - *_y, 2));
            if (d_min > d) {
                d_min = d;
                is_normal = true;
                id_normal = i;
            }
        }
    }

    float x1_minus_x0 = _path->at(id_normal - 0).x - _path->at(id_normal - 1).x;
    float y1_minus_y0 = _path->at(id_normal - 0).y - _path->at(id_normal - 1).y;
    float x0_minus_x = _path->at(id_normal - 1).x - *_x;
    float y0_minus_y = _path->at(id_normal - 1).y - *_y;

    float a = x1_minus_x0 * x1_minus_x0 + y1_minus_y0 * y1_minus_y0;
    float b = 2 * (x1_minus_x0 * x0_minus_x + y1_minus_y0 * y0_minus_y);
    float c = x0_minus_x * x0_minus_x + y0_minus_y * y0_minus_y - _lookahead_distance * _lookahead_distance;
    float d = b * b - 4 * a * c;

    if (d < 0) {
        goal_x = _path->at(id_normal - 0).x;
        goal_y = _path->at(id_normal - 0).y;
    } else {
        float t1 = (-b + sqrt(d)) / (2 * a);
        float t2 = (-b - sqrt(d)) / (2 * a);

        if (t1 >= 0 && t1 <= 1) {
            goal_x = _path->at(id_normal - 1).x + t1 * x1_minus_x0;
            goal_y = _path->at(id_normal - 1).y + t1 * y1_minus_y0;
        } else if (t2 >= 0 && t2 <= 1) {
            goal_x = _path->at(id_normal - 1).x + t2 * x1_minus_x0;
            goal_y = _path->at(id_normal - 1).y + t2 * y1_minus_y0;
        } else {
            goal_x = _path->at(id_normal - 0).x;
            goal_y = _path->at(id_normal - 0).y;
        }
    }
}

void
HelpPurePursuit::_update_icr() {
    float a = atan2(goal_y - *_y, goal_x - *_x) - *_theta;
    icr_radius = (sqrt(pow(goal_y - *_y, 2) + pow(goal_x - *_x, 2))) / (2 * sin(a));
    icr_x = *_x + icr_radius * cos(*_theta + M_PI_2);
    icr_y = *_y + icr_radius * sin(*_theta + M_PI_2);
}

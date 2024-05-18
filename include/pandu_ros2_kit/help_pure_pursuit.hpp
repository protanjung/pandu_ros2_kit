/**
 * @file help_pure_pursuit.hpp
 * @author Pandu Surya Tantra (pandustantra@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-06
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef HELP_PURE_PURSUIT_HPP_
#define HELP_PURE_PURSUIT_HPP_

#include "geometry_msgs/msg/point.hpp"
#include "pandu_ros2_kit/help_logger.hpp"
#include "rclcpp/rclcpp.hpp"

class HelpPurePursuit {
  private:
    bool _is_initialized = false;
    bool _is_ackermann = false;
    bool _is_holonomic = false;

    HelpLogger _logger;

    float* _x;
    float* _y;
    float* _theta;
    std::vector<geometry_msgs::msg::Point>* _path;

    float _wheel_base;
    float _lookahead_distance;

    void _update_goal();
    void _update_icr();

  public:
    float goal_x, goal_y;
    float icr_radius, icr_x, icr_y;

    HelpPurePursuit();

    bool is_initialized();

    bool init_as_ackermann(float* x, float* y, float* theta, std::vector<geometry_msgs::msg::Point>* path,
                           float wheel_base, float lookahead_distance);
    bool init_as_holonomic(float* x, float* y, float* theta, std::vector<geometry_msgs::msg::Point>* path,
                           float lookahead_distance);

    void update();

    void set_path(std::vector<geometry_msgs::msg::Point>* path);
    void set_lookahead_distance(float lookahead_distance);
    void set_wheel_base(float wheel_base);
    float get_lookahead_distance();
    float get_wheel_base();

    void process_ackermann(float* steering_angle);
    void process_holonomic(float* lin_vel, float* ang_vel, float max_lin_vel, float max_ang_vel);
};

#endif // HELP_PURE_PURSUIT_HPP_
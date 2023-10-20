#ifndef HELP_MARKER_HPP_
#define HELP_MARKER_HPP_

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"

class HelpMarker {
 public:
  //-----Node
  rclcpp::Node::SharedPtr _node;
  //-----Publisher
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker;

  bool is_initialized = false;

  HelpMarker();

  void init(rclcpp::Node::SharedPtr node);

  void arrow(
      std::string frame_id,
      std::string ns,
      int id,
      std::vector<geometry_msgs::msg::Point> points,
      std::vector<float> color,
      float scale_shaft,
      float scale_head);
  void cube(
      std::string frame_id,
      std::string ns,
      int id,
      geometry_msgs::msg::Point position,
      geometry_msgs::msg::Quaternion orientation,
      std::vector<float> color,
      float scale_x,
      float scale_y,
      float scale_z);
  void sphere(
      std::string frame_id,
      std::string ns,
      int id,
      geometry_msgs::msg::Point position,
      geometry_msgs::msg::Quaternion orientation,
      std::vector<float> color,
      float scale_x,
      float scale_y,
      float scale_z);
  void cylinder(
      std::string frame_id,
      std::string ns,
      int id,
      geometry_msgs::msg::Point position,
      geometry_msgs::msg::Quaternion orientation,
      std::vector<float> color,
      float scale_x,
      float scale_y,
      float scale_z);
  void line_strip(
      std::string frame_id,
      std::string ns,
      int id,
      std::vector<geometry_msgs::msg::Point> points,
      std::vector<float> color,
      float scale);
  void line_list(
      std::string frame_id,
      std::string ns,
      int id,
      std::vector<geometry_msgs::msg::Point> points,
      std::vector<float> color,
      float scale);
  void cube_list(
      std::string frame_id,
      std::string ns,
      int id,
      std::vector<geometry_msgs::msg::Point> points,
      geometry_msgs::msg::Quaternion orientation,
      std::vector<float> color,
      float scale_x,
      float scale_y,
      float scale_z);
  void sphere_list(
      std::string frame_id,
      std::string ns,
      int id,
      std::vector<geometry_msgs::msg::Point> points,
      geometry_msgs::msg::Quaternion orientation,
      std::vector<float> color,
      float scale_x,
      float scale_y,
      float scale_z);
  void points(
      std::string frame_id,
      std::string ns,
      int id,
      std::vector<geometry_msgs::msg::Point> points,
      std::vector<float> color,
      float scale_width,
      float scale_height);
  void text_view_facing(
      std::string frame_id,
      std::string ns,
      int id,
      std::string text,
      geometry_msgs::msg::Point position,
      std::vector<float> color,
      float scale);

  void array_to_color(std::vector<float> array, std_msgs::msg::ColorRGBA &color);
  void color_to_array(std_msgs::msg::ColorRGBA color, std::vector<float> &array);
};

#endif  // HELP_MARKER_HPP_
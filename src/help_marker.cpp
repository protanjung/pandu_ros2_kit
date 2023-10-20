#include "pandu_ros2_kit/help_marker.hpp"

HelpMarker::HelpMarker() {}

// =====================================

void HelpMarker::init(rclcpp::Node::SharedPtr node) {
  if (is_initialized) { return; }

  // ----Node
  _node = node;
  // ----Publisher
  pub_marker = _node->create_publisher<visualization_msgs::msg::Marker>("marker", rclcpp::SensorDataQoS());

  is_initialized = true;
}

// =====================================

void HelpMarker::arrow(
    std::string frame_id,
    std::string ns,
    int id,
    std::vector<geometry_msgs::msg::Point> points,
    std::vector<float> color,
    float scale_shaft,
    float scale_head) {
  if (!is_initialized || id == 0 || points.size() != 2) { return; }

  std_msgs::msg::ColorRGBA _color;
  array_to_color(color, _color);

  visualization_msgs::msg::Marker msg_marker;
  // ----
  msg_marker.header.frame_id = frame_id;
  msg_marker.header.stamp = _node->now();
  msg_marker.ns = ns;
  msg_marker.id = abs(id);
  // ----
  msg_marker.type = visualization_msgs::msg::Marker::ARROW;
  msg_marker.action = id > 0 ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
  // ----
  msg_marker.pose.orientation.w = 1.0;
  msg_marker.scale.x = scale_shaft;
  msg_marker.scale.y = scale_head;
  msg_marker.color = _color;
  msg_marker.frame_locked = true;
  // ----
  msg_marker.points = points;
  // ----
  pub_marker->publish(msg_marker);
}

void HelpMarker::cube(
    std::string frame_id,
    std::string ns,
    int id,
    geometry_msgs::msg::Point position,
    geometry_msgs::msg::Quaternion orientation,
    std::vector<float> color,
    float scale_x,
    float scale_y,
    float scale_z) {
  if (!is_initialized || id == 0) { return; }

  std_msgs::msg::ColorRGBA _color;
  array_to_color(color, _color);

  visualization_msgs::msg::Marker msg_marker;
  // ----
  msg_marker.header.frame_id = frame_id;
  msg_marker.header.stamp = _node->now();
  msg_marker.ns = ns;
  msg_marker.id = abs(id);
  // ----
  msg_marker.type = visualization_msgs::msg::Marker::CUBE;
  msg_marker.action = id > 0 ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
  // ----
  msg_marker.pose.position = position;
  msg_marker.pose.orientation = orientation;
  msg_marker.scale.x = scale_x;
  msg_marker.scale.y = scale_y;
  msg_marker.scale.z = scale_z;
  msg_marker.color = _color;
  msg_marker.frame_locked = true;
  // ----
  pub_marker->publish(msg_marker);
}

void HelpMarker::sphere(
    std::string frame_id,
    std::string ns,
    int id,
    geometry_msgs::msg::Point position,
    geometry_msgs::msg::Quaternion orientation,
    std::vector<float> color,
    float scale_x,
    float scale_y,
    float scale_z) {
  if (!is_initialized || id == 0) { return; }

  std_msgs::msg::ColorRGBA _color;
  array_to_color(color, _color);

  visualization_msgs::msg::Marker msg_marker;
  // ----
  msg_marker.header.frame_id = frame_id;
  msg_marker.header.stamp = _node->now();
  msg_marker.ns = ns;
  msg_marker.id = abs(id);
  // ----
  msg_marker.type = visualization_msgs::msg::Marker::SPHERE;
  msg_marker.action = id > 0 ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
  // ----
  msg_marker.pose.position = position;
  msg_marker.pose.orientation = orientation;
  msg_marker.scale.x = scale_x;
  msg_marker.scale.y = scale_y;
  msg_marker.scale.z = scale_z;
  msg_marker.color = _color;
  msg_marker.frame_locked = true;
  // ----
  pub_marker->publish(msg_marker);
}

void HelpMarker::cylinder(
    std::string frame_id,
    std::string ns,
    int id,
    geometry_msgs::msg::Point position,
    geometry_msgs::msg::Quaternion orientation,
    std::vector<float> color,
    float scale_x,
    float scale_y,
    float scale_z) {
  if (!is_initialized || id == 0) { return; }

  std_msgs::msg::ColorRGBA _color;
  array_to_color(color, _color);

  visualization_msgs::msg::Marker msg_marker;
  // ----
  msg_marker.header.frame_id = frame_id;
  msg_marker.header.stamp = _node->now();
  msg_marker.ns = ns;
  msg_marker.id = abs(id);
  // ----
  msg_marker.type = visualization_msgs::msg::Marker::CYLINDER;
  msg_marker.action = id > 0 ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
  // ----
  msg_marker.pose.position = position;
  msg_marker.pose.orientation = orientation;
  msg_marker.scale.x = scale_x;
  msg_marker.scale.y = scale_y;
  msg_marker.scale.z = scale_z;
  msg_marker.color = _color;
  msg_marker.frame_locked = true;
  // ----
  pub_marker->publish(msg_marker);
}

void HelpMarker::line_strip(
    std::string frame_id,
    std::string ns,
    int id,
    std::vector<geometry_msgs::msg::Point> points,
    std::vector<float> color,
    float scale) {
  if (!is_initialized || id == 0 || points.size() < 2) { return; }

  std_msgs::msg::ColorRGBA _color;
  array_to_color(color, _color);

  visualization_msgs::msg::Marker msg_marker;
  // ----
  msg_marker.header.frame_id = frame_id;
  msg_marker.header.stamp = _node->now();
  msg_marker.ns = ns;
  msg_marker.id = abs(id);
  // ----
  msg_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  msg_marker.action = id > 0 ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
  // ----
  msg_marker.pose.orientation.w = 1.0;
  msg_marker.scale.x = scale;
  msg_marker.color = _color;
  msg_marker.frame_locked = true;
  // ----
  msg_marker.points = points;
  // ----
  pub_marker->publish(msg_marker);
}

void HelpMarker::line_list(
    std::string frame_id,
    std::string ns,
    int id,
    std::vector<geometry_msgs::msg::Point> points,
    std::vector<float> color,
    float scale) {
  if (!is_initialized || id == 0 || points.size() % 2 != 0) { return; }

  std_msgs::msg::ColorRGBA _color;
  array_to_color(color, _color);

  visualization_msgs::msg::Marker msg_marker;
  // ----
  msg_marker.header.frame_id = frame_id;
  msg_marker.header.stamp = _node->now();
  msg_marker.ns = ns;
  msg_marker.id = abs(id);
  // ----
  msg_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  msg_marker.action = id > 0 ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
  // ----
  msg_marker.pose.orientation.w = 1.0;
  msg_marker.scale.x = scale;
  msg_marker.color = _color;
  msg_marker.frame_locked = true;
  // ----
  msg_marker.points = points;
  // ----
  pub_marker->publish(msg_marker);
}

void HelpMarker::cube_list(
    std::string frame_id,
    std::string ns,
    int id,
    std::vector<geometry_msgs::msg::Point> points,
    geometry_msgs::msg::Quaternion orientation,
    std::vector<float> color,
    float scale_x,
    float scale_y,
    float scale_z) {
  if (!is_initialized || id == 0 || points.size() < 1) { return; }

  std_msgs::msg::ColorRGBA _color;
  array_to_color(color, _color);

  visualization_msgs::msg::Marker msg_marker;
  // ----
  msg_marker.header.frame_id = frame_id;
  msg_marker.header.stamp = _node->now();
  msg_marker.ns = ns;
  msg_marker.id = abs(id);
  // ----
  msg_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
  msg_marker.action = id > 0 ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
  // ----
  msg_marker.pose.orientation = orientation;
  msg_marker.scale.x = scale_x;
  msg_marker.scale.y = scale_y;
  msg_marker.scale.z = scale_z;
  msg_marker.color = _color;
  msg_marker.frame_locked = true;
  // ----
  msg_marker.points = points;
  // ----
  pub_marker->publish(msg_marker);
}

void HelpMarker::sphere_list(
    std::string frame_id,
    std::string ns,
    int id,
    std::vector<geometry_msgs::msg::Point> points,
    geometry_msgs::msg::Quaternion orientation,
    std::vector<float> color,
    float scale_x,
    float scale_y,
    float scale_z) {
  if (!is_initialized || id == 0 || points.size() < 1) { return; }

  std_msgs::msg::ColorRGBA _color;
  array_to_color(color, _color);

  visualization_msgs::msg::Marker msg_marker;
  // ----
  msg_marker.header.frame_id = frame_id;
  msg_marker.header.stamp = _node->now();
  msg_marker.ns = ns;
  msg_marker.id = abs(id);
  // ----
  msg_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  msg_marker.action = id > 0 ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
  // ----
  msg_marker.pose.orientation = orientation;
  msg_marker.scale.x = scale_x;
  msg_marker.scale.y = scale_y;
  msg_marker.scale.z = scale_z;
  msg_marker.color = _color;
  msg_marker.frame_locked = true;
  // ----
  msg_marker.points = points;
  // ----
  pub_marker->publish(msg_marker);
}

void HelpMarker::points(
    std::string frame_id,
    std::string ns,
    int id,
    std::vector<geometry_msgs::msg::Point> points,
    std::vector<float> color,
    float scale_width,
    float scale_height) {
  if (!is_initialized || id == 0 || points.size() < 1) { return; }

  std_msgs::msg::ColorRGBA _color;
  array_to_color(color, _color);

  visualization_msgs::msg::Marker msg_marker;
  // ----
  msg_marker.header.frame_id = frame_id;
  msg_marker.header.stamp = _node->now();
  msg_marker.ns = ns;
  msg_marker.id = abs(id);
  // ----
  msg_marker.type = visualization_msgs::msg::Marker::POINTS;
  msg_marker.action = id > 0 ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
  // ----
  msg_marker.pose.orientation.w = 1.0;
  msg_marker.scale.x = scale_width;
  msg_marker.scale.y = scale_height;
  msg_marker.color = _color;
  msg_marker.frame_locked = true;
  // ----
  msg_marker.points = points;
  // ----
  pub_marker->publish(msg_marker);
}

void HelpMarker::text_view_facing(
    std::string frame_id,
    std::string ns,
    int id,
    std::string text,
    geometry_msgs::msg::Point position,
    std::vector<float> color,
    float scale) {
  if (!is_initialized || id == 0) { return; }

  std_msgs::msg::ColorRGBA _color;
  array_to_color(color, _color);

  visualization_msgs::msg::Marker msg_marker;
  // ----
  msg_marker.header.frame_id = frame_id;
  msg_marker.header.stamp = _node->now();
  msg_marker.ns = ns;
  msg_marker.id = abs(id);
  // ----
  msg_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  msg_marker.action = id > 0 ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
  // ----
  msg_marker.pose.position = position;
  msg_marker.scale.z = scale;
  msg_marker.color = _color;
  msg_marker.frame_locked = true;
  // ----
  msg_marker.text = text;
  // ----
  pub_marker->publish(msg_marker);
}

// =====================================

void HelpMarker::array_to_color(std::vector<float> array, std_msgs::msg::ColorRGBA &color) {
  if (array.size() != 4) { return; }

  color.r = array[0];
  color.g = array[1];
  color.b = array[2];
  color.a = array[3];
}

void HelpMarker::color_to_array(std_msgs::msg::ColorRGBA color, std::vector<float> &array) {
  array.resize(4);

  array[0] = color.r;
  array[1] = color.g;
  array[2] = color.b;
  array[3] = color.a;
}
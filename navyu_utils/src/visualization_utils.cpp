// Copyright 2025 RyuYamamoto.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License

#include "navyu_utils/visualization_utils.hpp"

namespace visualization_utils
{

std::string to_string(const double & num, const int & decimal_place)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(decimal_place) << num;
  return oss.str();
}

geometry_msgs::msg::Vector3 create_scale(const float & x, const float & y, const float & z)
{
  geometry_msgs::msg::Vector3 scale;
  scale.x = x;
  scale.y = y;
  scale.z = z;
  return scale;
}

std_msgs::msg::ColorRGBA create_color(
  const float & r, const float & g, const float & b, const float & a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

visualization_msgs::msg::Marker create_cube_marker(
  const rclcpp::Time & stamp, const std::string & frame_id, const geometry_msgs::msg::Pose & pose,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::Marker cube_marker;
  cube_marker.header.frame_id = frame_id;
  cube_marker.header.stamp = stamp;
  cube_marker.type = visualization_msgs::msg::Marker::CUBE;
  cube_marker.action = visualization_msgs::msg::Marker::ADD;
  cube_marker.lifetime = rclcpp::Duration::from_seconds(0.3);
  cube_marker.pose = pose;
  cube_marker.color = color;
  cube_marker.scale = scale;

  return cube_marker;
}

visualization_msgs::msg::Marker create_text_marker(
  const rclcpp::Time & stamp, const std::string & frame_id, const std::string & text,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::Marker text_marker;
  text_marker.header.frame_id = frame_id;
  text_marker.header.stamp = stamp;
  text_marker.ns = "text";
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::msg::Marker::ADD;
  text_marker.scale = scale;
  text_marker.color = color;
  text_marker.lifetime = rclcpp::Duration::from_seconds(0.3);
  text_marker.text = text;

  return text_marker;
}

visualization_msgs::msg::Marker create_foot_print_marker(
  const rclcpp::Time & stamp, const std::string & frame_id, const int id,
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Polygon & foot_print,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::Marker foot_print_marker;
  foot_print_marker.header.frame_id = frame_id;
  foot_print_marker.header.stamp = stamp;
  foot_print_marker.ns = "footprint";
  foot_print_marker.id = id;
  foot_print_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  foot_print_marker.action = visualization_msgs::msg::Marker::ADD;
  foot_print_marker.pose = pose;
  foot_print_marker.scale = scale;
  foot_print_marker.color = color;
  foot_print_marker.lifetime = rclcpp::Duration::from_seconds(0.3);

  for (const auto & p : foot_print.points) {
    geometry_msgs::msg::Point point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    foot_print_marker.points.emplace_back(point);
  }

  if (!foot_print_marker.points.empty()) {
    foot_print_marker.points.emplace_back(foot_print_marker.points.front());
  }

  return foot_print_marker;
}

visualization_msgs::msg::MarkerArray foot_print_visualization(
  const rclcpp::Time & stamp, const std::string & frame_id,
  const std::vector<geometry_msgs::msg::Pose> & poses,
  const geometry_msgs::msg::Polygon & foot_print, const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::MarkerArray foot_print_marker_array;

  for (std::size_t i = 0; i < poses.size(); i++) {
    visualization_msgs::msg::Marker foot_print_marker =
      create_foot_print_marker(stamp, frame_id, i, poses[i], foot_print, scale, color);

    for (const auto & p : foot_print.points) {
      geometry_msgs::msg::Point point;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      foot_print_marker.points.emplace_back(point);
    }

    if (!foot_print_marker.points.empty()) {
      foot_print_marker.points.emplace_back(foot_print_marker.points.front());
    }

    foot_print_marker_array.markers.emplace_back(foot_print_marker);
  }

  return foot_print_marker_array;
}

}  // namespace visualization_utils

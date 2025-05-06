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

#ifndef NAVYU_UTILS__VISUALIZATION_UTILS_HPP_
#define NAVYU_UTILS__VISUALIZATION_UTILS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/polygon.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace visualization_utils
{

geometry_msgs::msg::Vector3 create_scale(const float & x, const float & y, const float & z);

std_msgs::msg::ColorRGBA create_color(
  const float & r, const float & g, const float & b, const float & a);

visualization_msgs::msg::Marker create_cube_marker(
  const rclcpp::Time & stamp, const std::string & frame_id, const geometry_msgs::msg::Pose & pose,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color);

visualization_msgs::msg::Marker create_text_marker(
  const rclcpp::Time & stamp, const std::string & frame_id, const std::string & text,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color);

visualization_msgs::msg::Marker create_foot_print_marker(
  const rclcpp::Time & stamp, const std::string & frame_id, const int id,
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Polygon & foot_print,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color);

visualization_msgs::msg::MarkerArray foot_print_visualization(
  const rclcpp::Time & stamp, const std::vector<geometry_msgs::msg::Pose> & poses,
  const geometry_msgs::msg::Polygon & foot_print, const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color);

std::string to_string(const double & num, const int & decimal_place);

}  // namespace visualization_utils

#endif

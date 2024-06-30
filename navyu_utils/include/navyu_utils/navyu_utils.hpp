// Copyright 2024 RyuYamamoto.
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

#ifndef NAVYU_UTIULS__NAVYU_UTILS_HPP_
#define NAVYU_UTIULS__NAVYU_UTILS_HPP_

#include <Eigen/Core>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace navyu_utils
{

bool get_robot_pose(
  const std::string global_frame, const std::string robot_frame, tf2_ros::Buffer & tf_buffer,
  geometry_msgs::msg::Pose & robot_pose);

double calculation_distance(const geometry_msgs::msg::Pose p1, const geometry_msgs::msg::Pose p2);

double normalized_radian(const double radian);

}  // namespace navyu_utils

#endif

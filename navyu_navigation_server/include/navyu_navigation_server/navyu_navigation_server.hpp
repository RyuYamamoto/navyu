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

#ifndef NAVYU_NAVIGATION_SERVER__NAVYU_NAVIGATION_SERVER_HPP_
#define NAVYU_NAVIGATION_SERVER__NAVYU_NAVIGATION_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <navyu_msgs/srv/goal.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class NavyuNavigationServer : public rclcpp::Node
{
public:
  explicit NavyuNavigationServer(const rclcpp::NodeOptions & node_options);
  ~NavyuNavigationServer();

  void callback_goal_pose(const geometry_msgs::msg::PoseStamped msg);

  void update();

  bool get_robot_pose(geometry_msgs::msg::Pose & robot_pose);

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscriber_;

  rclcpp::Client<navyu_msgs::srv::Goal>::SharedPtr goal_client_;

  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  double update_frequency_;
  double is_planning_{false};
  std::string map_frame_;
  std::string base_frame_;

  geometry_msgs::msg::PoseStamped goal_pose_;
  geometry_msgs::msg::PoseStamped current_pose_;
};

#endif

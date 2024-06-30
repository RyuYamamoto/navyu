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

#ifndef NAVYU_PLANNER__NAVYU_GLOBAL_PLANNER_HPP_
#define NAVYU_PLANNER__NAVYU_GLOBAL_PLANNER_HPP_

#include "navyu_path_planner/astar_planner.hpp"
#include "navyu_path_planner/smoother.hpp"
#include "navyu_utils/navyu_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class NavyuGlobalPlanner : public rclcpp::Node
{
public:
  explicit NavyuGlobalPlanner(const rclcpp::NodeOptions & node_options);
  ~NavyuGlobalPlanner();

  bool plan(
    const geometry_msgs::msg::Pose start, const geometry_msgs::msg::Pose goal,
    std::vector<Node2D *> & path);

  void callback_costmap(const nav_msgs::msg::OccupancyGrid & msg);
  void callback_goal_pose(const geometry_msgs::msg::PoseStamped & msg);

  void publish_path(std::vector<Node2D *> path);

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_publisher_;

  tf2_ros::Buffer tf_buffer_{get_clock()};
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  std::shared_ptr<AstarPlanner> planner_;
  std::shared_ptr<Smoother> smoother_;

  std::string map_frame_;
  std::string base_frame_;
};

#endif  // NAVYU_PLANNER__NAVYU_GLOBAL_PLANNER_HPP_

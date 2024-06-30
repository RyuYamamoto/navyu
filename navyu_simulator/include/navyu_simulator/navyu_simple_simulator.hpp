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

#ifndef NAVYU_SIMULATOR__NAVYU_SIMPLE_SIMULATOR_HPP_
#define NAVYU_SIMULATOR__NAVYU_SIMPLE_SIMULATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class NavyuSimpleSimulator : public rclcpp::Node
{
public:
  NavyuSimpleSimulator(const rclcpp::NodeOptions & node_options);
  ~NavyuSimpleSimulator();

  void callback_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & msg);
  void callback_cmd_vel(const geometry_msgs::msg::Twist & msg);

  void motion();

  void process();

  void publish_tf(
    const geometry_msgs::msg::Pose pose, const std::string base_frame,
    const std::string child_frame, const rclcpp::Time stamp);

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pubilsher_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  geometry_msgs::msg::PoseStamped current_pose_;

  geometry_msgs::msg::Twist cmd_vel_msg_;

  rclcpp::Time current_stamp_;
  rclcpp::Time previous_stamp_;

  double period_;
  std::string base_frame_id_;

  std::array<double, 36> covariance_;
  std::vector<std::string> joint_names_;
};

#endif  // NAVYU_SIMULATOR__NAVYU_SIMPLE_SIMULATOR_HPP_

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

#include "navyu_simulator/navyu_simple_simulator.hpp"

#include "navyu_utils/quaternion_utils.hpp"

#include <kdl/frames.hpp>
#include <rclcpp/clock.hpp>

#include <chrono>

NavyuSimpleSimulator::NavyuSimpleSimulator(const rclcpp::NodeOptions & node_options)
: Node("navyu_simple_simulator", node_options)
{
  period_ = declare_parameter<double>("period");
  base_frame_id_ = declare_parameter<std::string>("base_frame_id");

  joint_names_ = declare_parameter<std::vector<std::string>>("joint_names");

  std::vector<double> covariance = declare_parameter<std::vector<double>>("covariance");
  for (std::size_t idx = 0; idx < covariance.size(); idx++) covariance_[idx] = covariance[idx];

  twist_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&NavyuSimpleSimulator::callback_cmd_vel, this, std::placeholders::_1));
  current_pose_publisher_ =
    create_publisher<geometry_msgs::msg::PoseStamped>("localization_pose", 10);
  joint_state_pubilsher_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 5);

  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  current_stamp_ = rclcpp::Clock().now();
  previous_stamp_ = current_stamp_;

  timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 * period_)),
    std::bind(&NavyuSimpleSimulator::process, this));
}

NavyuSimpleSimulator::~NavyuSimpleSimulator()
{
}

void NavyuSimpleSimulator::callback_cmd_vel(const geometry_msgs::msg::Twist & msg)
{
  cmd_vel_msg_ = msg;
}

void NavyuSimpleSimulator::callback_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & msg)
{
  current_pose_.pose = msg->pose.pose;
  current_pose_.header = msg->header;
}

void NavyuSimpleSimulator::process()
{
  current_stamp_ = rclcpp::Clock().now();
  const double dt = (current_stamp_ - previous_stamp_).seconds();

  // update current pose
  geometry_msgs::msg::Vector3 diff_angular;
  diff_angular.z = cmd_vel_msg_.angular.z * dt;
  geometry_msgs::msg::Vector3 current_euler =
    quaternion_utils::convert_quaternion_to_euler(current_pose_.pose.orientation);

  // TODO: use rotation matrix
  geometry_msgs::msg::Vector3 diff_translation;
  if (std::fabs(cmd_vel_msg_.angular.z) < 1e-06) {
    diff_translation.x = cmd_vel_msg_.linear.x * std::cos(current_euler.z) * dt;
    diff_translation.y = cmd_vel_msg_.linear.x * std::sin(current_euler.z) * dt;
  } else {
    const double trans_vel = (cmd_vel_msg_.linear.x / cmd_vel_msg_.angular.z);
    diff_translation.x =
      trans_vel * (std::sin(current_euler.z + diff_angular.z) - std::sin(current_euler.z));
    diff_translation.y =
      trans_vel * (-std::cos(current_euler.z + diff_angular.z) + std::cos(current_euler.z));
  }

  current_pose_.pose.position.x += diff_translation.x;
  current_pose_.pose.position.y += diff_translation.y;
  current_euler.z += diff_angular.z;
  current_pose_.pose.orientation = quaternion_utils::convert_euler_to_quaternion(current_euler);

  // publish current pose
  current_pose_.header.stamp = current_stamp_;
  current_pose_.header.frame_id = "map";
  current_pose_publisher_->publish(current_pose_);

  // publish joint states
  sensor_msgs::msg::JointState joint_states_msg;
  joint_states_msg.header.stamp = current_stamp_;
  for (auto joint_name : joint_names_) {
    joint_states_msg.name.emplace_back(joint_name);
    joint_states_msg.position.emplace_back(0.0);
    joint_states_msg.velocity.emplace_back(0.0);
    joint_states_msg.effort.emplace_back(0.0);
  }
  joint_state_pubilsher_->publish(joint_states_msg);

  // publish tf
  publish_tf(current_pose_.pose, "map", base_frame_id_, current_stamp_);

  previous_stamp_ = current_stamp_;
}

void NavyuSimpleSimulator::publish_tf(
  const geometry_msgs::msg::Pose pose, const std::string base_frame, const std::string child_frame,
  const rclcpp::Time stamp)
{
  geometry_msgs::msg::TransformStamped transform;

  transform.header.stamp = stamp;
  transform.header.frame_id = base_frame;
  transform.child_frame_id = child_frame;
  transform.transform.translation.x = pose.position.x;
  transform.transform.translation.y = pose.position.y;
  transform.transform.translation.z = pose.position.z;
  transform.transform.rotation.x = pose.orientation.x;
  transform.transform.rotation.y = pose.orientation.y;
  transform.transform.rotation.z = pose.orientation.z;
  transform.transform.rotation.w = pose.orientation.w;

  broadcaster_->sendTransform(transform);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(NavyuSimpleSimulator)

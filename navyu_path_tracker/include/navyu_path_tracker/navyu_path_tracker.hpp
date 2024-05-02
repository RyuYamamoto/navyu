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

#ifndef NAVYU_PATH_TRACEKR__NAVYU_PATH_TRACKER_HPP_
#define NAVYU_PATH_TRACEKR__NAVYU_PATH_TRACKER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class NavyuPathTracker : public rclcpp::Node
{
public:
  explicit NavyuPathTracker(const rclcpp::NodeOptions & node_options);
  ~NavyuPathTracker();

  void process();

  void callback_path(const nav_msgs::msg::Path::SharedPtr msg);

  bool get_robot_pose(geometry_msgs::msg::Pose & robot_pose);

  void publish_velocity_command(const double v, const double w);

  double calculation_distance(
    const geometry_msgs ::msg::Pose p1, const geometry_msgs::msg::Pose p2);

  visualization_msgs::msg::Marker create_marker(
    const geometry_msgs::msg::Pose pose, const geometry_msgs::msg::Vector3 scale,
    const std_msgs::msg::ColorRGBA color);

  inline std_msgs::msg::ColorRGBA create_color(
    const float a, const float r, const float g, const float b)
  {
    std_msgs::msg::ColorRGBA color;
    color.a = a;
    color.r = r;
    color.g = g;
    color.b = b;
    return color;
  }
  inline geometry_msgs::msg::Vector3 create_scale(const float x, const float y, const float z)
  {
    geometry_msgs::msg::Vector3 scale;
    scale.x = x;
    scale.y = y;
    scale.z = z;
    return scale;
  }
  inline geometry_msgs::msg::Vector3 convert_quaternion_to_euler(
    const geometry_msgs::msg::Quaternion quaternion)
  {
    geometry_msgs::msg::Vector3 euler;

    tf2::Quaternion quat(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    tf2::Matrix3x3 matrix(quat);
    matrix.getRPY(euler.x, euler.y, euler.z);

    return euler;
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr nearest_point_marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_point_marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  nav_msgs::msg::Path path_;

  std::string base_frame_;
  std::string map_frame_;

  double update_frequency_;

  // translational and rotational speed
  double limit_v_speed_;
  double limit_w_speed_;

  double v_;
  double w_;
};

#endif

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

#include "navyu_path_tracker/navyu_path_tracker.hpp"

NavyuPathTracker::NavyuPathTracker(const rclcpp::NodeOptions & node_options)
: Node("navyu_path_tracker", node_options)
{
  update_frequency_ = declare_parameter<double>("update_frequency");
  map_frame_ = declare_parameter<std::string>("map_frame");
  base_frame_ = declare_parameter<std::string>("base_frame");

  limit_v_speed_ = declare_parameter<double>("limit_v_speed");
  limit_w_speed_ = declare_parameter<double>("limit_w_speed");

  path_subscriber_ = create_subscription<nav_msgs::msg::Path>(
    "path", 1, std::bind(&NavyuPathTracker::callback_path, this, std::placeholders::_1));

  cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  nearest_point_marker_publisher_ =
    create_publisher<visualization_msgs::msg::Marker>("nearest_point_marker", 10);
  target_point_marker_publisher_ =
    create_publisher<visualization_msgs::msg::Marker>("target_point_marker", 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1.0 / update_frequency_ * 1000)),
    std::bind(&NavyuPathTracker::process, this));
}
NavyuPathTracker::~NavyuPathTracker()
{
}

double NavyuPathTracker::calculation_distance(
  const geometry_msgs::msg::Pose p1, const geometry_msgs::msg::Pose p2)
{
  const double dx = p1.position.x - p2.position.x;
  const double dy = p1.position.y - p2.position.y;
  return std::hypot(dx, dy);
}

void NavyuPathTracker::publish_velocity_command(const double v, const double w)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = v;
  twist.angular.z = w;
  cmd_vel_publisher_->publish(twist);
}

visualization_msgs::msg::Marker NavyuPathTracker::create_marker(
  const geometry_msgs::msg::Pose pose, const geometry_msgs::msg::Vector3 scale,
  const std_msgs::msg::ColorRGBA color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = map_frame_;
  marker.header.stamp = now();
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.pose = pose;
  marker.scale = scale;
  marker.color = color;
  return marker;
}

void NavyuPathTracker::process()
{
  // path is not set
  if (path_.poses.empty()) return;

  // get robot pose from tf
  geometry_msgs::msg::Pose robot_pose;
  if (!get_robot_pose(robot_pose)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Can not get Robot Pose.");
    return;
  }

  int nearest_point_index = 0;
  std::vector<double> distance_list;
  for (std::size_t idx = nearest_point_index; idx < path_.poses.size(); idx++) {
    distance_list.emplace_back(calculation_distance(robot_pose, path_.poses[idx].pose));
  }
  std::vector<double>::iterator itr = std::min_element(distance_list.begin(), distance_list.end());
  nearest_point_index = std::distance(distance_list.begin(), itr);

  if (path_.poses.size() - 1 <= nearest_point_index) {
    v_ = w_ = 0.0;
    publish_velocity_command(v_, w_);
    return;
  }

  // limit linear velocity
  double a = 1.0 * (limit_v_speed_ - v_);

  // update look ahead distance
  const double look_ahead_filter = 0.1 * v_ + 0.3;

  int target_point_index = nearest_point_index;
  while (calculation_distance(robot_pose, path_.poses[target_point_index].pose) <
         look_ahead_filter) {
    if (path_.poses.size() <= target_point_index + 1) break;
    target_point_index++;
  }
  if (path_.poses.size() < target_point_index) target_point_index = path_.poses.size();

  // update steer
  geometry_msgs::msg::Vector3 euler = convert_quaternion_to_euler(robot_pose.orientation);
  const double dx = path_.poses[target_point_index].pose.position.x - robot_pose.position.x;
  const double dy = path_.poses[target_point_index].pose.position.y - robot_pose.position.y;
  const double alpha = std::atan2(dy, dx) - euler.z;
  w_ = 2 * v_ * std::sin(alpha) / look_ahead_filter;

  // update current linear velocity
  v_ += (a * (1.0 / update_frequency_));

  // publish velocity command
  v_ = std::clamp(v_, -limit_v_speed_, limit_v_speed_);
  w_ = std::clamp(w_, -limit_w_speed_, limit_w_speed_);
  publish_velocity_command(v_, w_);

  // publish debug marker
  nearest_point_marker_publisher_->publish(create_marker(
    path_.poses[nearest_point_index].pose, create_scale(0.1, 0.1, 0.1),
    create_color(1.0, 0.0, 1.0, 0.0)));
  target_point_marker_publisher_->publish(create_marker(
    path_.poses[target_point_index].pose, create_scale(0.1, 0.1, 0.1),
    create_color(1.0, 1.0, 1.0, 0.0)));
}

void NavyuPathTracker::callback_path(const nav_msgs::msg::Path::SharedPtr msg)
{
  path_ = *msg;
}

bool NavyuPathTracker::get_robot_pose(geometry_msgs::msg::Pose & robot_pose)
{
  geometry_msgs::msg::TransformStamped robot_pose_frame;

  try {
    robot_pose_frame = tf_buffer_.lookupTransform(
      map_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Can not get Transform " << map_frame_ << " to " << base_frame_);
    return false;
  }

  robot_pose.position.x = robot_pose_frame.transform.translation.x;
  robot_pose.position.y = robot_pose_frame.transform.translation.y;
  robot_pose.position.z = robot_pose_frame.transform.translation.z;
  robot_pose.orientation = robot_pose_frame.transform.rotation;

  return true;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(NavyuPathTracker)
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

#include "navyu_safety_limiter/navyu_safety_limiter.hpp"

NavyuSafetyLimiter::NavyuSafetyLimiter(const rclcpp::NodeOptions & node_options)
: Node("navyu_safety_limiter", node_options)
{
  base_frame_ = this->declare_parameter<std::string>("base_frame");

  use_radius_foot_print_ = this->declare_parameter<bool>("use_radius_foot_print");

  predict_step_ = this->declare_parameter<int>("predict_step");
  predict_time_ = this->declare_parameter<double>("predict_time");
  margin_ = this->declare_parameter<double>("margin");
  alpha_ = this->declare_parameter<double>("alpha");

  foot_print_radius_ = this->declare_parameter<double>("foot_print_radius");
  std::vector<double> foot_print = this->declare_parameter<std::vector<double>>("foot_print");
  foot_print_ = create_foot_print(foot_print);

  costmap_ = std::make_shared<CostmapHelper>();

  // create subscriber
  cmd_vel_in_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_in", 10,
    std::bind(&NavyuSafetyLimiter::callback_cmd_vel, this, std::placeholders::_1));
  costmap_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "local_costmap", 10,
    std::bind(&NavyuSafetyLimiter::callback_costmap, this, std::placeholders::_1));

  // create publisher
  predict_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("predict_path", 10);
  cmd_vel_out_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", 10);
  stop_wall_marker_publisher_ =
    this->create_publisher<visualization_msgs::msg::Marker>("stop_wall_marker", 10);
  current_state_marker_publisher_ =
    this->create_publisher<visualization_msgs::msg::Marker>("current_state_marker", 5);
  foot_print_publisher_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("foot_print", 10);

  const double hz = this->declare_parameter<double>("hz");
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>(1000.0 / hz)),
    std::bind(&NavyuSafetyLimiter::process, this));
}

NavyuSafetyLimiter::~NavyuSafetyLimiter()
{
}

void NavyuSafetyLimiter::process()
{
  if (costmap_->get_costmap() == nullptr) {
    cmd_vel_out_publisher_->publish(cmd_vel_in_);
    return;
  }

  geometry_msgs::msg::Pose robot_pose;
  if (!navyu_utils::get_robot_pose("map", base_frame_, tf_buffer_, robot_pose)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Can not get frame.");
    cmd_vel_out_publisher_->publish(cmd_vel_in_);
    return;
  }

  const double linear_velocity = cmd_vel_in_.linear.x;
  if (
    std::fabs(linear_velocity) < std::numeric_limits<double>::epsilon() &&
    std::fabs(cmd_vel_in_.angular.z) < std::numeric_limits<double>::epsilon()) {
    cmd_vel_out_publisher_->publish(cmd_vel_in_);
    return;
  }

  // collision check simulation
  double collision_distance;
  std::vector<geometry_msgs::msg::Pose> predict_poses;
  bool has_collision = predict(robot_pose, cmd_vel_in_, collision_distance, predict_poses);

  auto sgn = [](double val) -> double { return (val > 0.0) ? 1.0 : ((val < 0.0) ? -1.0 : 0.0); };

  const double d_col = std::max(0.0, collision_distance - margin_);
  const double v_lim =
    sgn(d_col) * sgn(linear_velocity) * std::sqrt(2.0 * alpha_ * std::fabs(d_col));
  double w_lim = cmd_vel_in_.angular.z;
  if (std::numeric_limits<double>::epsilon() < std::abs(linear_velocity)) {
    w_lim = cmd_vel_in_.angular.z * (v_lim / linear_velocity);
  }
  cmd_vel_in_.linear.x = std::min(v_lim, cmd_vel_in_.linear.x);
  cmd_vel_in_.angular.z = std::min(w_lim, cmd_vel_in_.angular.z);

  // Visualization
  publish_debug_marker(has_collision, collision_distance, predict_poses);

  // publish target velocity
  cmd_vel_out_publisher_->publish(cmd_vel_in_);
}

void NavyuSafetyLimiter::publish_debug_marker(
  const bool & has_collision, const double & collision_distance,
  const std::vector<geometry_msgs::msg::Pose> & predict_poses)
{
  const auto current_stamp = this->now();

  if (has_collision) {
    geometry_msgs::msg::Vector3 scale = visualization_utils::create_scale(0.03, 3.0, 1.0);
    std_msgs::msg::ColorRGBA color = visualization_utils::create_color(1.0, 0.0, 0.0, 0.7);
    visualization_msgs::msg::Marker stop_wall_marker = visualization_utils::create_cube_marker(
      current_stamp, "map", predict_poses.back(), scale, color);
    stop_wall_marker_publisher_->publish(stop_wall_marker);

    std::string text = "Collision:" + visualization_utils::to_string(collision_distance, 2) + "m";
    scale = visualization_utils::create_scale(1.0, 1.0, 0.3);
    color = visualization_utils::create_color(1.0, 0.0, 0.0, 1.0);
    visualization_msgs::msg::Marker text_marker =
      visualization_utils::create_text_marker(current_stamp, base_frame_, text, scale, color);
    current_state_marker_publisher_->publish(text_marker);
  }

  visualization_msgs::msg::MarkerArray foot_print_array;
  const auto current_time_stamp = this->now();
  for (std::size_t i = 0; i < predict_poses.size(); i++) {
    const geometry_msgs::msg::Vector3 scale = visualization_utils::create_scale(0.02, 0.0, 0.0);
    std_msgs::msg::ColorRGBA color = visualization_utils::create_color(0.0, 0.0, 1.0, 1.0);
    if (i == predict_poses.size() - 1 && has_collision) {
      color = visualization_utils::create_color(1.0, 0.0, 0.0, 1.0);
    }
    visualization_msgs::msg::Marker foot_print = visualization_utils::create_foot_print_marker(
      current_time_stamp, "map", i, predict_poses[i], foot_print_, scale, color);
    foot_print_array.markers.emplace_back(foot_print);
  }

  foot_print_publisher_->publish(foot_print_array);
}

bool NavyuSafetyLimiter::predict(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Twist & cmd_vel_in,
  double & collision_distance, std::vector<geometry_msgs::msg::Pose> & predict_poses)
{
  const double dt = predict_time_ / static_cast<double>(predict_step_);
  collision_distance = 0.0;

  geometry_msgs::msg::Pose predict_pose = pose;
  geometry_msgs::msg::Pose last_pose = pose;
  double yaw = tf2::getYaw(predict_pose.orientation);

  predict_poses.emplace_back(predict_pose);

  bool has_collision = false;
  for (int i = 0; i < predict_step_; i++) {
    const double dx =
      (cmd_vel_in.linear.x * std::cos(yaw) - cmd_vel_in.linear.y * std::sin(yaw)) * dt;
    const double dy =
      (cmd_vel_in.linear.x * std::sin(yaw) + cmd_vel_in.linear.y * std::cos(yaw)) * dt;
    yaw += cmd_vel_in.angular.z * dt;

    collision_distance += std::hypot(dx, dy);

    predict_pose.position.x += dx;
    predict_pose.position.y += dy;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    predict_pose.orientation = tf2::toMsg(q);

    int mx, my;
    if (!costmap_->convert_world_to_map(predict_pose.position.x, predict_pose.position.y, mx, my)) {
      continue;
    }

    const geometry_msgs::msg::Polygon foot_print = transform_foot_print(predict_pose, foot_print_);
    if (costmap_->obstacle_collision_check(foot_print)) {
      collision_distance -= std::hypot(dx, dy);
      has_collision = true;
      break;
    }

    last_pose = predict_pose;
    predict_poses.emplace_back(predict_pose);
  }

  return has_collision;
}

geometry_msgs::msg::Polygon NavyuSafetyLimiter::create_foot_print(std::vector<double> vertex)
{
  geometry_msgs::msg::Polygon polygon;

  for (std::size_t i = 0; i < vertex.size(); i += 2) {
    geometry_msgs::msg::Point32 point;
    point.x = vertex[i];
    point.y = vertex[i + 1];
    polygon.points.emplace_back(point);
  }

  return polygon;
}

geometry_msgs::msg::Polygon NavyuSafetyLimiter::transform_foot_print(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Polygon & foot_print)
{
  const double x = pose.position.x;
  const double y = pose.position.y;
  const double yaw = tf2::getYaw(pose.orientation);

  geometry_msgs::msg::Polygon new_foot_print;
  new_foot_print.points.resize(foot_print.points.size());
  for (std::size_t i = 0; i < foot_print.points.size(); i++) {
    geometry_msgs::msg::Point32 point;
    point.x = x + (foot_print.points[i].x * std::cos(yaw) - foot_print.points[i].y * std::sin(yaw));
    point.y = y + (foot_print.points[i].x * std::sin(yaw) + foot_print.points[i].y * std::cos(yaw));
    new_foot_print.points[i] = point;
  }
  return new_foot_print;
}

void NavyuSafetyLimiter::callback_costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  costmap_->set_costmap(msg);
}

void NavyuSafetyLimiter::callback_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vel_in_ = *msg;
}

bool NavyuSafetyLimiter::get_transform(
  const std::string target_frame, const std::string source_frame,
  geometry_msgs::msg::TransformStamped & frame)
{
  try {
    frame = tf_buffer_.lookupTransform(
      target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    return false;
  }
  return true;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(NavyuSafetyLimiter)

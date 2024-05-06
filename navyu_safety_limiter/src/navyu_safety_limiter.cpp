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
  base_frame_ = declare_parameter<std::string>("base_frame");
  slowdown_ratio_ = declare_parameter<double>("slowdown_ratio");
  collision_points_threshold_in_polygon_ =
    declare_parameter<int>("collision_points_threshold_in_polygon");

  std::vector<double> slowdown_polygon =
    this->declare_parameter<std::vector<double>>("slowdown_polygon");
  std::vector<double> stop_polygon = this->declare_parameter<std::vector<double>>("stop_polygon");

  // initialize polygon
  Polygon polygon;
  polygon.robot_radius_ = declare_parameter<double>("robot_radius");
  polygon.length_ = declare_parameter<double>("length");
  polygon.scale_ = declare_parameter<double>("scale");

  // slowdown
  polygon.name_ = "slowdown_polygon";
  polygon.type_ = SLOWDOWN;
  polygon.update_dynamic_ = true;
  polygon.polygon_ = create_polygon(slowdown_polygon);
  polygons_.emplace_back(polygon);
  // stop
  polygon.name_ = "stop_polygon";
  polygon.type_ = STOP;
  polygon.update_dynamic_ = false;
  polygon.polygon_ = create_polygon(stop_polygon);
  polygons_.emplace_back(polygon);

  // create subscriber
  scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10, std::bind(&NavyuSafetyLimiter::callback_laser_scan, this, std::placeholders::_1));
  cmd_vel_in_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_in", 10,
    std::bind(&NavyuSafetyLimiter::callback_cmd_vel, this, std::placeholders::_1));

  // create publisher
  cmd_vel_out_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", 10);
  current_state_marker_publisher_ =
    this->create_publisher<visualization_msgs::msg::Marker>("current_state_marker", 5);
  for (auto polygon : polygons_) {
    polygon_pubilsher_[polygon.name_] =
      create_publisher<geometry_msgs::msg::PolygonStamped>(polygon.name_, 10);
  }
}

NavyuSafetyLimiter::~NavyuSafetyLimiter()
{
}

geometry_msgs::msg::Polygon NavyuSafetyLimiter::create_polygon(std::vector<double> vertex)
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

bool NavyuSafetyLimiter::check_collision_points_in_polygon(
  const geometry_msgs::msg::Polygon polygon,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr collison_points)
{
  int collision_point_num = 0;
  for (auto point : collison_points->points) {
    double sum_angle = 0;
    for (std::size_t idx = 0; idx < polygon.points.size(); ++idx) {
      const double vec_1_x = polygon.points[idx].x - point.x;
      const double vec_1_y = polygon.points[idx].y - point.y;
      const double vec_2_x = polygon.points[(idx + 1) % polygon.points.size()].x - point.x;
      const double vec_2_y = polygon.points[(idx + 1) % polygon.points.size()].y - point.y;

      const double vec_1 = vec_1_x * vec_2_x + vec_1_y * vec_2_y;
      const double vec_2 = vec_1_y * vec_2_x - vec_1_x * vec_2_y;

      const double angle = std::atan2(vec_2, vec_1);
      sum_angle += angle;
    }

    if ((2 * M_PI - std::fabs(sum_angle)) < std::numeric_limits<double>::epsilon())
      collision_point_num++;
  }

  return (collision_points_threshold_in_polygon_ <= collision_point_num) ? true : false;
}

void NavyuSafetyLimiter::callback_cmd_vel(const geometry_msgs::msg::Twist msg)
{
  cmd_vel_in_ = msg;

  visualization_msgs::msg::Marker text;

  // check collision
  geometry_msgs::msg::Twist cmd_vel_out = msg;

  if (current_state_ == "SLOWDOWN") {
    cmd_vel_out.linear.x = slowdown_ratio_ * msg.linear.x;
    cmd_vel_out.linear.y = slowdown_ratio_ * msg.linear.y;
    cmd_vel_out.angular.z = slowdown_ratio_ * msg.angular.z;
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 0.0;
  } else if (current_state_ == "STOP") {
    cmd_vel_out.linear.x = 0.0;
    cmd_vel_out.linear.y = 0.0;
    cmd_vel_out.angular.z = 0.0;
    text.color.r = 1.0;
    text.color.g = 0.0;
    text.color.b = 0.0;
  } else {
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
  }

  // visualization safety status
  text.header.frame_id = base_frame_;
  text.header.stamp = now();
  text.ns = "text";
  text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text.action = visualization_msgs::msg::Marker::ADD;
  text.pose.position.x = -0.5;
  text.scale.z = 0.3;
  text.text = current_state_;
  text.color.a = 1.0;
  current_state_marker_publisher_->publish(text);

  // publish filter cmd_vel
  cmd_vel_out_publisher_->publish(cmd_vel_out);
  current_control_twist_ = cmd_vel_out;
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

void NavyuSafetyLimiter::callback_laser_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 cloud_msg;
  projection_.projectLaser(*msg, cloud_msg);

  // convert point cloud from ros msg
  pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloud_msg, *laser_cloud);

  geometry_msgs::msg::TransformStamped transform;
  if (!get_transform(base_frame_, msg->header.frame_id, transform)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Can not get frame.");
    return;
  }

  // transform point cloud
  const Eigen::Affine3d affine = tf2::transformToEigen(transform);
  const Eigen::Matrix4f matrix = affine.matrix().cast<float>();
  pcl::transformPointCloud(*laser_cloud, *transform_cloud, matrix);

  current_state_ = "NONE";

  // check collision
  for (auto polygon : polygons_) {
    geometry_msgs::msg::Polygon dynamic_polygon;
    if (polygon.type_ == STOP)
      dynamic_polygon = polygon.polygon_;
    else
      dynamic_polygon = polygon.update_polygon(cmd_vel_in_);

    geometry_msgs::msg::PolygonStamped polygon_msg;
    polygon_msg.header.stamp = now();
    polygon_msg.header.frame_id = base_frame_;
    polygon_msg.polygon = dynamic_polygon;
    polygon_pubilsher_[polygon.name_]->publish(polygon_msg);

    if (!check_collision_points_in_polygon(dynamic_polygon, transform_cloud)) continue;

    if (polygon.type_ == SLOWDOWN) {
      current_state_ = "SLOWDOWN";
    } else if (polygon.type_ == STOP) {
      current_state_ = "STOP";
    }
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(NavyuSafetyLimiter)

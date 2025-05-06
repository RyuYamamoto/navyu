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

#ifndef NAVYU_SAFETY_LIMITER__NAVYU_SAFETY_LIMITER_HPP_
#define NAVYU_SAFETY_LIMITER__NAVYU_SAFETY_LIMITER_HPP_

#include "navyu_utils/costmap_helper.hpp"
#include "navyu_utils/navyu_utils.hpp"
#include "navyu_utils/visualization_utils.hpp"

#include <laser_geometry/laser_geometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>

enum Type {
  STOP = 0,
  SLOWDOWN,
};

struct Polygon
{
  std::string name_;
  bool update_dynamic_{false};
  double length_;
  double scale_;
  double robot_radius_;
  geometry_msgs::msg::Polygon polygon_;
  Type type_;

  geometry_msgs::msg::Polygon update_polygon(geometry_msgs::msg::Twist twist)
  {
    if (!update_dynamic_) return polygon_;

    geometry_msgs::msg::Polygon polygon;

    geometry_msgs::msg::Point32 p1, p2, p3, p4;
    p1.x = 0.0;
    p1.y = robot_radius_;
    p2.x = 0.0;
    p2.y = -1 * robot_radius_;
    p3.x = length_ + twist.linear.x * scale_;
    p3.y = -1 * robot_radius_;
    p4.x = length_ + twist.linear.x * scale_;
    p4.y = robot_radius_;

    polygon.points.emplace_back(p1);
    polygon.points.emplace_back(p2);
    polygon.points.emplace_back(p3);
    polygon.points.emplace_back(p4);

    return polygon;
  }
};

using PolygonPublisher = rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr;

class NavyuSafetyLimiter : public rclcpp::Node
{
public:
  explicit NavyuSafetyLimiter(const rclcpp::NodeOptions & node_options);
  ~NavyuSafetyLimiter();

  void process();

  void callback_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void callback_costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  void publish_debug_marker(
    const bool & has_collision, const double & collision_distance,
    const std::vector<geometry_msgs::msg::Pose> & predict_poses);

  // collision check simulation
  bool predict(
    const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Twist & cmd_vel_in,
    double & collision_distance, std::vector<geometry_msgs::msg::Pose> & predict_poses);

  geometry_msgs::msg::Polygon create_foot_print(std::vector<double> vertex);

  bool get_transform(
    const std::string target_frame, const std::string source_frame,
    geometry_msgs::msg::TransformStamped & frame);

  geometry_msgs::msg::Polygon transform_foot_print(
    const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Polygon & foot_print);

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_in_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predict_path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr current_state_marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr stop_wall_marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr foot_print_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener listener_{tf_buffer_};

  geometry_msgs::msg::Twist cmd_vel_in_;

  std::shared_ptr<CostmapHelper> costmap_;

  geometry_msgs::msg::Polygon foot_print_;

  std::string base_frame_;

  int predict_step_;
  double predict_time_;
  double foot_print_radius_;
  double margin_;
  double alpha_;

  bool use_radius_foot_print_;
};

#endif

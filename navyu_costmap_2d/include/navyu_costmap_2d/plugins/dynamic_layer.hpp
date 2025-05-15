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

#ifndef NAVYU_COSTMAP_2D__PLUGINS__DYNAMIC_LAYER_HPP_
#define NAVYU_COSTMAP_2D__PLUGINS__DYNAMIC_LAYER_HPP_

#include "navyu_costmap_2d/plugins/layer.hpp"

#include <laser_geometry/laser_geometry.hpp>
#include <pcl_ros/transforms.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class DynamicLayer : public Layer
{
public:
  explicit DynamicLayer(rclcpp::Node * node) : Layer(node) {}

  void configure() override
  {
    min_laser_range_ = node_->declare_parameter<double>("dynamic_layer.min_laser_range");
    max_laser_range_ = node_->declare_parameter<double>("dynamic_layer.max_laser_range");

    update_map_origin_ = node_->declare_parameter<bool>("dynamic_layer.update_map_origin", false);

    global_frame_ = node_->declare_parameter<std::string>("dynamic_layer.global_frame");
    const std::string scan_topic =
      node_->declare_parameter<std::string>("dynamic_layer.scan_topic");
    scan_subscriber_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS().keep_last(10),
      std::bind(&DynamicLayer::callback, this, std::placeholders::_1));
  }

  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    pose_ = msg;
  }

  void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { scan_ = msg; }

  bool update(nav_msgs::msg::OccupancyGrid & master_costmap) override
  {
    if (scan_ == nullptr) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "scan message is nullptr.");
      return false;
    }

    // scan range filter
    for (std::size_t i = 0; i < scan_->ranges.size(); i++) {
      if (scan_->ranges[i] < min_laser_range_ or max_laser_range_ < scan_->ranges[i])
        scan_->ranges[i] = std::numeric_limits<float>::quiet_NaN();
    }

    // convert laser scan to point cloud msg
    sensor_msgs::msg::PointCloud2 cloud_msg, transform_cloud_msg;
    projection_.projectLaser(*scan_, cloud_msg);
    if (cloud_msg.data.empty()) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "scan is empty.");
      return false;
    }
    // get robot pose
    geometry_msgs::msg::TransformStamped transform_frame;
    if (!get_transform(global_frame_, scan_->header.frame_id, transform_frame)) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Can not get frame.");
      return false;
    }

    // update map origin
    if (update_map_origin_) {
      const double size_x = master_costmap.info.width * master_costmap.info.resolution;
      const double size_y = master_costmap.info.height * master_costmap.info.resolution;
      master_costmap.info.origin.position.x =
        transform_frame.transform.translation.x - size_x / 2.0;
      master_costmap.info.origin.position.y =
        transform_frame.transform.translation.y - size_y / 2.0;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_msg, *laser_cloud);

    // transform laser cloud
    if (scan_->header.frame_id != global_frame_) {
      const Eigen::Affine3d affine = tf2::transformToEigen(transform_frame);
      const Eigen::Matrix4f matrix = affine.matrix().cast<float>();
      pcl::transformPointCloud(*laser_cloud, *transform_cloud, matrix);
    } else {
      transform_cloud = laser_cloud;
    }

    // get costmap info
    const double resolution = master_costmap.info.resolution;
    const int width = master_costmap.info.width;
    const int height = master_costmap.info.height;
    const double origin_x = master_costmap.info.origin.position.x;
    const double origin_y = master_costmap.info.origin.position.y;
    master_costmap.data.resize(width * height);

    // calculation grid index
    for (auto cloud : transform_cloud->points) {
      const int ix = static_cast<int>((cloud.x - origin_x) / resolution);
      const int iy = static_cast<int>((cloud.y - origin_y) / resolution);

      if (0 <= ix and ix < width and 0 <= iy and iy < height) {
        int index = ix + width * iy;
        master_costmap.data[index] = LETHAL_COST;
      }
    }

    return true;
  }

  bool get_transform(
    const std::string target_frame, const std::string source_frame,
    geometry_msgs::msg::TransformStamped & frame)
  {
    try {
      frame = tf_buffer_->lookupTransform(
        target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
      return false;
    }
    return true;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;

  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_;

  laser_geometry::LaserProjection projection_;

  std::string global_frame_;

  double min_laser_range_;
  double max_laser_range_;

  bool update_map_origin_;
};

#endif

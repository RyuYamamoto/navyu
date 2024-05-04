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

#ifndef NAVYU_COSTMAP_2D__NAVYU_COSTMAP_2D_HPP_
#define NAVYU_COSTMAP_2D__NAVYU_COSTMAP_2D_HPP_

#include "navyu_costmap_2d/plugins/layer.hpp"

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class NavyuCostmap2D : public rclcpp::Node
{
public:
  explicit NavyuCostmap2D(const rclcpp::NodeOptions & node_options);
  ~NavyuCostmap2D();

  void update();

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;

  std::vector<std::string> plugins_;
  std::map<std::string, std::shared_ptr<Layer>> layer_function_;

  double update_frequency_;
  std::string base_frame_id_;
  std::string map_frame_id_;
};

#endif  // NAVYU_COSTMAP_2D__NAVYU_COSTMAP_2D_HPP_

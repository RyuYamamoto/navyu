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

#include "navyu_costmap_2d/navyu_costmap_2d.hpp"

#include "navyu_costmap_2d/plugins/dynamic_layer.hpp"
#include "navyu_costmap_2d/plugins/inflation_layer.hpp"
#include "navyu_costmap_2d/plugins/static_layer.hpp"

#include <tf2/time.h>
#include <tf2_ros/transform_listener.h>

NavyuCostmap2D::NavyuCostmap2D(const rclcpp::NodeOptions & node_options)
: Node("navyu_costmap_2d", node_options)
{
  update_frequency_ = declare_parameter<double>("update_frequency");
  global_frame_id_ = declare_parameter<std::string>("global_frame_id");

  resolution_ = declare_parameter<double>("resolution", 0.05);
  origin_x_ = declare_parameter<double>("origin_x", 0.0);
  origin_y_ = declare_parameter<double>("origin_y", 0.0);
  size_x_ = declare_parameter<int>("width", 5);
  size_y_ = declare_parameter<int>("height", 5);

  master_costmap_.header.frame_id = global_frame_id_;
  master_costmap_.info.resolution = resolution_;
  master_costmap_.info.width = static_cast<int32_t>(size_x_ / resolution_);
  master_costmap_.info.height = static_cast<int32_t>(size_y_ / resolution_);
  master_costmap_.info.origin.position.x = origin_x_ - static_cast<double>(size_x_) / 2.0;
  master_costmap_.info.origin.position.y = origin_y_ - static_cast<double>(size_y_) / 2.0;
  master_costmap_.data.resize(master_costmap_.info.width * master_costmap_.info.height);

  plugins_ = declare_parameter<std::vector<std::string>>("plugins");

  // create all costmap plugins instance
  layer_function_["static_layer"] = std::make_shared<StaticLayer>(this);
  layer_function_["dynamic_layer"] = std::make_shared<DynamicLayer>(this);
  layer_function_["inflation_layer"] = std::make_shared<InflationLayer>(this);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  for (auto plugin : plugins_) {
    layer_function_[plugin]->initialize(tf_buffer_.get(), listener_.get());
    layer_function_[plugin]->configure();
  }

  rclcpp::QoS qos(10);
  qos.transient_local();
  qos.reliable();
  qos.keep_last(1);

  costmap_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", qos);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1.0 / update_frequency_ * 1000)),
    std::bind(&NavyuCostmap2D::update, this));
}

NavyuCostmap2D::~NavyuCostmap2D()
{
}

void NavyuCostmap2D::update()
{
  master_costmap_.data.clear();

  // update costmap
  for (auto & plugin : plugins_) {
    if (!layer_function_[plugin]->update(master_costmap_)) {
      RCLCPP_ERROR_STREAM(get_logger(), "Can not update costmap. " << plugin.c_str());
      return;
    }
  }

  if (master_costmap_.data.empty()) {
    RCLCPP_ERROR_STREAM(get_logger(), "master costmap is empty");
    return;
  }

  master_costmap_.header.stamp = now();

  costmap_publisher_->publish(master_costmap_);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(NavyuCostmap2D)

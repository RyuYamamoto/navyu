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
  base_frame_id_ = declare_parameter<std::string>("base_frame_id");
  map_frame_id_ = declare_parameter<std::string>("map_frame_id");

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
  nav_msgs::msg::OccupancyGrid master_costmap;

  // update costmap
  for (auto & plugin : plugins_) {
    layer_function_[plugin]->update(master_costmap);
  }

  if (master_costmap.data.empty()) {
    RCLCPP_ERROR_STREAM(get_logger(), "master costmap is empty");
    return;
  }

  master_costmap.header.stamp = now();
  master_costmap.header.frame_id = map_frame_id_;

  costmap_publisher_->publish(master_costmap);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(NavyuCostmap2D)

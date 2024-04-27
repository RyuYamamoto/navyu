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

#include "navyu_costmap_2d/plugins/inflation_layer.hpp"
#include "navyu_costmap_2d/plugins/static_layer.hpp"

NavyuCostmap2D::NavyuCostmap2D(const rclcpp::NodeOptions & node_options)
: Node("navyu_costmap_2d", node_options)
{
  update_frequency_ = declare_parameter<double>("update_frequency");
  base_frame_id_ = declare_parameter<std::string>("base_frame_id");
  map_frame_id_ = declare_parameter<std::string>("map_frame_id");

  plugins_ = declare_parameter<std::vector<std::string>>("plugins");

  // create all costmap plugins instance
  layer_function_["static_layer"] = std::make_shared<StaticLayer>(this);
  layer_function_["inflation_layer"] = std::make_shared<InflationLayer>(this);

  for (auto plugin : plugins_) layer_function_[plugin]->initialize();

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
  nav_msgs::msg::OccupancyGrid::SharedPtr master_costmap;

  // update costmap
  for (auto & plugin : plugins_) {
    layer_function_[plugin]->update(master_costmap);
  }

  if (master_costmap == nullptr) {
    RCLCPP_ERROR_STREAM(get_logger(), "master costmap is null");
    return;
  }

  master_costmap->header.stamp = now();
  master_costmap->header.frame_id = map_frame_id_;

  costmap_publisher_->publish(*master_costmap);
}

geometry_msgs::msg::TransformStamped NavyuCostmap2D::get_transform(
  const std::string target_frame, const std::string source_frame)
{
  geometry_msgs::msg::TransformStamped frame_transform;
  try {
    frame_transform = tf_buffer_.lookupTransform(
      target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    frame_transform.header.stamp = rclcpp::Clock().now();
    frame_transform.header.frame_id = target_frame;
    frame_transform.child_frame_id = source_frame;
    frame_transform.transform.translation.x = 0.0;
    frame_transform.transform.translation.y = 0.0;
    frame_transform.transform.translation.z = 0.0;
    frame_transform.transform.rotation.w = 1.0;
    frame_transform.transform.rotation.x = 0.0;
    frame_transform.transform.rotation.y = 0.0;
    frame_transform.transform.rotation.z = 0.0;
  }
  return frame_transform;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(NavyuCostmap2D)

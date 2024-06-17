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

#ifndef NAVYU_COSTMAP_2D__PLUGINS__STATIC_LAYER_HPP_
#define NAVYU_COSTMAP_2D__PLUGINS__STATIC_LAYER_HPP_

#include "navyu_costmap_2d/plugins/layer.hpp"

class StaticLayer : public Layer
{
public:
  explicit StaticLayer(rclcpp::Node * node) : Layer(node) {}

  void configure() override
  {
    rclcpp::QoS qos(10);
    qos.transient_local();
    qos.reliable();
    qos.keep_last(1);

    const std::string map_topic = node_->declare_parameter<std::string>("static_layer.map_topic");
    map_subscriber_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, qos, std::bind(&StaticLayer::callback_map, this, std::placeholders::_1));
  }

  void callback_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { map_ = *msg; }

  bool update(nav_msgs::msg::OccupancyGrid & master_costmap) override
  {
    if (map_.data.empty()) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "map is empty.");
      return false;
    }
    master_costmap = map_;
    return true;
  }

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;

  nav_msgs::msg::OccupancyGrid map_;
};

#endif  // NAVYU_COSTMAP_2D__PLUGINS__STATIC_LAYER_HPP_

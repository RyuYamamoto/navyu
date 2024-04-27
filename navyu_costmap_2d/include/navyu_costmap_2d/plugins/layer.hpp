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

#ifndef NAVYU_COSTMAP_2D__PLUGINS__LAYER_HPP_
#define NAVYU_COSTMAP_2D__PLUGINS__LAYER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

static constexpr int8_t LETHAL_COST = 100;
static constexpr int8_t INSCRIBED_COST = 99;

class Layer
{
public:
  explicit Layer(rclcpp::Node * node) { node_ = node; }
  virtual ~Layer() {}
  virtual void initialize() = 0;
  virtual void update(nav_msgs::msg::OccupancyGrid::SharedPtr & master_costmap) = 0;

private:
  rclcpp::Node * node_;
};

#endif  // NAVYU_COSTMAP_2D__PLUGINS__LAYER_HPP_

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

#ifndef NAVYU_COSTMAP_2D__PLUGINS__INFLATION_LAYER_HPP_
#define NAVYU_COSTMAP_2D__PLUGINS__INFLATION_LAYER_HPP_

#include "navyu_costmap_2d/plugins/layer.hpp"

class InflationLayer : public Layer
{
public:
  explicit InflationLayer(rclcpp::Node * node) : Layer(node) { node_ = node; }

  void initialize() override
  {
    node_->declare_parameter<double>("inflation_layer.inflation_radius");
    node_->get_parameter("inflation_layer.inflation_radius", inflation_radius_);
    node_->declare_parameter<double>("inflation_layer.robot_radius");
    node_->get_parameter("inflation_layer.robot_radius", robot_radius_);
  }

  void update(nav_msgs::msg::OccupancyGrid::SharedPtr & master_costmap) override
  {
    if (master_costmap == nullptr) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "costmap is nullptr.");
      return;
    }

    const int size_x = master_costmap->info.width;
    const int size_y = master_costmap->info.height;

    for (int map_y = 0; map_y < size_y; map_y++) {
      for (int map_x = 0; map_x < size_x; map_x++) {
        int map_index = map_x + size_x * map_y;

        if (master_costmap->data[map_index] == LETHAL_COST) {
          expand_cost(master_costmap, map_x, map_y);
        }
      }
    }
  }

  void expand_cost(nav_msgs::msg::OccupancyGrid::SharedPtr & costmap, const int mx, const int my)
  {
    const int size_x = costmap->info.width;
    const double resolution = costmap->info.resolution;
    const int cell_inflation_radius = std::ceil(inflation_radius_ / resolution);

    const int min_x = mx - cell_inflation_radius;
    const int max_x = mx + cell_inflation_radius;
    const int min_y = my - cell_inflation_radius;
    const int max_y = my + cell_inflation_radius;

    for (int y = min_y; y < max_y; y++) {
      for (int x = min_x; x < max_x; x++) {
        const double distance = std::hypot(x - mx, y - my);
        const int index = x + size_x * y;
        const auto old_cost = costmap->data[index];

        if (distance * resolution < robot_radius_) {
          costmap->data[index] = std::max(INSCRIBED_COST, old_cost);
        } else {
          if (cell_inflation_radius < distance) continue;
          if (costmap->data[index] == -1) continue;
          const int8_t new_cost =
            std::exp(-1 * 3.0 * (distance * resolution - robot_radius_)) * INSCRIBED_COST - 1;
          costmap->data[index] = std::max(new_cost, old_cost);
        }
      }
    }
  }

private:
  rclcpp::Node * node_;

  double inflation_radius_;
  double robot_radius_;
};

#endif  // NAVYU_COSTMAP_2D__PLUGINS__INFLATION_LAYER_HPP_

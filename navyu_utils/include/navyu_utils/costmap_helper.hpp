// Copyright 2025 RyuYamamoto.
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

#ifndef NAVYU_UTILS__COSTMAP_HELPER_HPP_
#define NAVYU_UTILS__COSTMAP_HELPER_HPP_

#include <geometry_msgs/msg/polygon.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <algorithm>
#include <cmath>

class CostmapHelper
{
public:
  CostmapHelper();
  ~CostmapHelper();

  inline void set_costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
  {
    costmap_ = map;
    size_x_ = costmap_->info.width;
    size_y_ = costmap_->info.height;
    resolution_ = costmap_->info.resolution;
    origin_x_ = costmap_->info.origin.position.x;
    origin_y_ = costmap_->info.origin.position.y;
  }
  inline int get_grid_index(int x, int y) { return x + size_x_ * y; }
  inline int get_cost(int index) { return costmap_->data[index]; }
  inline nav_msgs::msg::OccupancyGrid::SharedPtr get_costmap() { return costmap_; }

  bool convert_world_to_map(const double & wx, const double & wy, int & mx, int & my);
  bool convert_map_to_world(const int & mx, const int & my, double & wx, double & wy);
  bool check_bound(const int & mx, const int & my);

  bool is_obstacle_in_radius(const int & mx, const int & my, const double & radius);
  bool obstacle_collision_check(const geometry_msgs::msg::Polygon & foot_print);
  bool is_inside_polygon(
    const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Polygon & polygon);

private:
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;

  int size_x_;
  int size_y_;
  double resolution_;
  double origin_x_;
  double origin_y_;
};

#endif

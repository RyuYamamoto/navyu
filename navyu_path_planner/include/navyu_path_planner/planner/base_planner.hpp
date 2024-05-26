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

#ifndef NAVYU_PLANNER__BASE_PLANNER_HPP_
#define NAVYU_PLANNER__BASE_PLANNER_HPP_

#include <cmath>
#include <iostream>
#include <vector>

class BasePlanner
{
public:
  BasePlanner(int size_x, int size_y, double resolution, double origin_x, double origin_y)
  : size_x_(size_x),
    size_y_(size_y),
    resolution_(resolution),
    origin_x_(origin_x),
    origin_y_(origin_y)
  {
  }

  void set_size(int size_x, int size_y)
  {
    size_x_ = size_x;
    size_y_ = size_y;
  }

  void set_origin(double origin_x, double origin_y)
  {
    origin_x_ = origin_x;
    origin_y_ = origin_y;
  }

  void set_resolution(double resolution) { resolution_ = resolution; }

  void set_costmap(std::vector<int8_t> costmap) { costmap_ = costmap; }

  void set_lethal_cost_threshold(int8_t lethal_cost_threshold)
  {
    lethal_cost_threshold_ = lethal_cost_threshold;
  }

  int get_grid_index(int x, int y) { return x + size_x_ * y; }

  void convert_map_to_grid(double map_x, double map_y, int & grid_x, int & grid_y)
  {
    grid_x = static_cast<int>((map_x - origin_x_) / resolution_);
    grid_y = static_cast<int>((map_y - origin_y_) / resolution_);
  }

  void convert_grid_to_map(int grid_x, int grid_y, double & map_x, double & map_y)
  {
    map_x = origin_x_ + grid_x * resolution_;
    map_y = origin_y_ + grid_y * resolution_;
  }

protected:
  // costmap parameter
  int size_x_;
  int size_y_;
  double resolution_;
  double origin_x_;
  double origin_y_;

  std::vector<int8_t> costmap_;

  double lethal_cost_threshold_;
};

#endif  // NAVYU_PLANNER__BASE_PLANNER_HPP_

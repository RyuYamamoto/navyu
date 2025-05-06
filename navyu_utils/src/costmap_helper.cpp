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

#include "navyu_utils/costmap_helper.hpp"

#include <iostream>

CostmapHelper::CostmapHelper()
{
}

CostmapHelper::~CostmapHelper()
{
}

bool CostmapHelper::convert_world_to_map(const double & wx, const double & wy, int & mx, int & my)
{
  mx = static_cast<int>((wx - origin_x_) / resolution_);
  my = static_cast<int>((wy - origin_y_) / resolution_);

  return check_bound(mx, my);
}

bool CostmapHelper::convert_map_to_world(const int & mx, const int & my, double & wx, double & wy)
{
  if (!check_bound(mx, my)) {
    return false;
  }
  wx = origin_x_ + mx * resolution_;
  wy = origin_y_ + my * resolution_;
  return true;
}

bool CostmapHelper::check_bound(const int & mx, const int & my)
{
  if (mx < 0 || size_x_ <= mx || my < 0 || size_y_ <= my) {
    return false;
  }
  return true;
}

bool CostmapHelper::is_inside_polygon(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Polygon & polygon)
{
  double sum_angle = 0;
  for (std::size_t idx = 0; idx < polygon.points.size(); ++idx) {
    const double vec_1_x = polygon.points[idx].x - point.x;
    const double vec_1_y = polygon.points[idx].y - point.y;
    const double vec_2_x = polygon.points[(idx + 1) % polygon.points.size()].x - point.x;
    const double vec_2_y = polygon.points[(idx + 1) % polygon.points.size()].y - point.y;

    const double vec_1 = vec_1_x * vec_2_x + vec_1_y * vec_2_y;
    const double vec_2 = vec_1_y * vec_2_x - vec_1_x * vec_2_y;

    const double angle = std::atan2(vec_2, vec_1);
    sum_angle += angle;
  }

  return (2 * M_PI - std::fabs(sum_angle)) < std::numeric_limits<double>::epsilon();
}

bool CostmapHelper::obstacle_collision_check(const geometry_msgs::msg::Polygon & foot_print)
{
  int min_x = size_x_;
  int min_y = size_y_;
  int max_x = 0;
  int max_y = 0;
  for (const auto & point : foot_print.points) {
    int mx = static_cast<int>((point.x - origin_x_) / resolution_);
    int my = static_cast<int>((point.y - origin_y_) / resolution_);

    min_x = std::min(min_x, mx);
    min_y = std::min(min_y, my);
    max_x = std::max(max_x, mx);
    max_y = std::max(max_y, my);
  }

  min_x = std::max(0, min_x);
  min_y = std::max(0, min_y);
  max_x = std::min(size_x_ - 1, max_x);
  max_y = std::min(size_y_ - 1, max_y);

  for (int y = min_y; y <= max_y; ++y) {
    for (int x = min_x; x <= max_x; ++x) {
      geometry_msgs::msg::Point cell;
      if (!convert_map_to_world(x, y, cell.x, cell.y)) {
        continue;
      }
      if (is_inside_polygon(cell, foot_print)) {
        auto cost = get_cost(get_grid_index(x, y));
        if (99 < cost) {
          return true;
        }
      }
    }
  }
  return false;
}

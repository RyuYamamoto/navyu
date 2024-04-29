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

#ifndef NAVYU_PATH_PLANNER__SMOOTHER_HPP_
#define NAVYU_PATH_PLANNER__SMOOTHER_HPP_

#include <nav_msgs/msg/path.hpp>

#include <cmath>
#include <vector>

class Smoother
{
public:
  Smoother(double displacement_threshold) { displacement_threshold_ = displacement_threshold; }

  nav_msgs::msg::Path smooth(nav_msgs::msg::Path path)
  {
    std::vector<nav_msgs::msg::Path> segments_path = split_path_with_dist(path);
    nav_msgs::msg::Path optimized_path;

    auto factorial = [](int n) -> double {
      double result = 1.0;
      for (int i = 1; i <= n; i++) result *= i;
      return result;
    };

    auto binominal = [&](int n, int i) -> double {
      return factorial(n) / (factorial(i) * factorial(n - i));
    };

    auto bernstein = [&](int n, int i, double t) -> double {
      return binominal(n, i) * std::pow(t, i) * std::pow(1 - t, n - i);
    };

    for (auto segment : segments_path) {
      int path_size = segment.poses.size();
      double t_step = 1.0 / (double)path_size;

      for (double t = 0.0; t <= 1.0; t += t_step) {
        geometry_msgs::msg::PoseStamped pose;

        for (int idx = 0; idx < path_size; idx++) {
          pose.pose.position.x +=
            segment.poses[idx].pose.position.x * bernstein(path_size - 1, idx, t);
          pose.pose.position.y +=
            segment.poses[idx].pose.position.y * bernstein(path_size - 1, idx, t);
        }
        optimized_path.poses.emplace_back(pose);
      }
    }

    return optimized_path;
  }

  std::vector<nav_msgs::msg::Path> split_path_with_dist(nav_msgs::msg::Path path)
  {
    std::vector<nav_msgs::msg::Path> segments_path;

    auto calculate_distance =
      [](geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2) -> double {
      return std::hypot((p2.x - p1.x), (p2.y - p1.y));
    };

    double displacement = 0.0;
    nav_msgs::msg::Path current_segments_path;
    for (auto pose : path.poses) {
      current_segments_path.poses.emplace_back(pose);

      if (2 <= current_segments_path.poses.size()) {
        displacement += calculate_distance(
          current_segments_path.poses[current_segments_path.poses.size() - 2].pose.position,
          pose.pose.position);
      }

      if (5.0 < displacement) {
        segments_path.emplace_back(current_segments_path);
        current_segments_path.poses.clear();
        displacement = 0.0;
      }
    }

    current_segments_path.poses.emplace_back(path.poses[path.poses.size() - 1]);
    segments_path.emplace_back(current_segments_path);

    return segments_path;
  }

  std::vector<nav_msgs::msg::Path> split_path(nav_msgs::msg::Path path)
  {
    std::vector<nav_msgs::msg::Path> segments_path;
    nav_msgs::msg::Path current_segments_path;

    // detect cusp and split path
    for (int i = 0; i < path.poses.size() - 1; i++) {
      // cross product
      const double ux = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
      const double uy = path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y;
      const double vx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
      const double vy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
      const double cp = ux * vy - uy * vx;
      // const double cp = ux * vx + uy * vy;

      if (0.1 < std::abs(cp)) {
        segments_path.emplace_back(current_segments_path);
        current_segments_path.poses.clear();
      }
      current_segments_path.poses.emplace_back(path.poses[i]);
    }

    // insert goal node
    current_segments_path.poses.emplace_back(path.poses[path.poses.size() - 1]);
    segments_path.emplace_back(current_segments_path);

    return segments_path;
  }

private:
  double displacement_threshold_;
};

#endif

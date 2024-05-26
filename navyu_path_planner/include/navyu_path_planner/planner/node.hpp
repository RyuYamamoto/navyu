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

#ifndef NAVYU_PLANNER__NODE2D_HPP_
#define NAVYU_PLANNER__NODE2D_HPP_

#include <cmath>
#include <vector>

class Node2D
{
public:
  Node2D(int x = 0, int y = 0, double g = 0.0, Node2D * parent = NULL)
  : x_(x), y_(y), g_(g), h_(0.0), f_(0.0), parent_(parent)
  {
  }

  void cost(Node2D * n1, Node2D * n2)
  {
    h_ = std::hypot(n1->x_ - n2->x_, n1->y_ - n2->y_);
    f_ = h_ + g_;
  }

  void set_grid_index(int grid_index) { grid_index_ = grid_index; }

  std::vector<Node2D> get_motion()
  {
    std::vector<Node2D> motion{
      Node2D(1, 0, 1),
      Node2D(0, 1, 1),
      Node2D(-1, 0, 1),
      Node2D(0, -1, 1),
      Node2D(1, 1, std::sqrt(2)),
      Node2D(1, -1, std::sqrt(2)),
      Node2D(-1, 1, std::sqrt(2)),
      Node2D(-1, -1, std::sqrt(2)),
    };

    return motion;
  }

public:
  int x_;
  int y_;
  int grid_index_;
  Node2D * parent_;
  double g_;
  double h_;
  double f_;
};

#endif  // NAVYU_PLANNER__NODE2D_HPP_

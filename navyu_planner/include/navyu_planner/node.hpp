#ifndef NAVYU_PLANNER__NODE_HPP_
#define NAVYU_PLANNER__NODE_HPP_

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

#endif  // NAVYU_PLANNER__NODE_HPP_

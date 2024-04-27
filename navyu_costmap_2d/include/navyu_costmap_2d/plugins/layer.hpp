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

#ifndef NAVYU_COSTMAP_2D__PLUGINS__STATIC_LAYER_HPP_
#define NAVYU_COSTMAP_2D__PLUGINS__STATIC_LAYER_HPP_

#include "navyu_costmap_2d/plugins/layer.hpp"

#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>

class StaticLayer : public Layer
{
public:
  explicit StaticLayer(rclcpp::Node * node) : Layer(node) { node_ = node; }

  void initialize() override
  {
    rclcpp::QoS qos(10);
    qos.transient_local();
    qos.reliable();
    qos.keep_last(1);

    const std::string map_topic = node_->declare_parameter<std::string>("static_layer.map_topic");
    map_subscriber_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, qos, std::bind(&StaticLayer::callback_map, this, std::placeholders::_1));
  }

  void callback_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { costmap_ = msg; }

  void update(nav_msgs::msg::OccupancyGrid::SharedPtr & master_costmap) override
  {
    master_costmap = costmap_;
  }

private:
  rclcpp::Node * node_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;

  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
};

#endif  // NAVYU_COSTMAP_2D__PLUGINS__STATIC_LAYER_HPP_

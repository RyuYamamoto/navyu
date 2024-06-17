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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

static constexpr int8_t LETHAL_COST = 100;
static constexpr int8_t INSCRIBED_COST = 99;

class Layer
{
public:
  explicit Layer(rclcpp::Node * node) { node_ = node; }
  virtual ~Layer() {}
  void initialize(tf2_ros::Buffer * tf_buffer, tf2_ros::TransformListener * listener)
  {
    tf_buffer_ = tf_buffer;
    listener_ = listener;
  };
  virtual void configure() = 0;
  virtual bool update(nav_msgs::msg::OccupancyGrid & master_costmap) = 0;

protected:
  rclcpp::Node * node_;
  tf2_ros::Buffer * tf_buffer_;
  tf2_ros::TransformListener * listener_;
};

#endif  // NAVYU_COSTMAP_2D__PLUGINS__LAYER_HPP_

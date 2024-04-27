#ifndef NAVYU_COSTMAP_2D__NAVYU_COSTMAP_2D_HPP_
#define NAVYU_COSTMAP_2D__NAVYU_COSTMAP_2D_HPP_

#include "navyu_costmap_2d/plugins/layer.hpp"

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class NavyuCostmap2D : public rclcpp::Node
{
public:
  explicit NavyuCostmap2D(const rclcpp::NodeOptions & node_options);
  ~NavyuCostmap2D();

  void update();

private:
  geometry_msgs::msg::TransformStamped get_transform(
    const std::string target_frame, const std::string source_frame);

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::string> plugins_;
  std::map<std::string, std::shared_ptr<Layer>> layer_function_;

  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  double update_frequency_;
  std::string base_frame_id_;
  std::string map_frame_id_;
};

#endif  // NAVYU_COSTMAP_2D__NAVYU_COSTMAP_2D_HPP_

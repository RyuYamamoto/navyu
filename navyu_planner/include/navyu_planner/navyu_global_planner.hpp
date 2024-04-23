#ifndef NAVYU_PLANNER__NAVYU_GLOBAL_PLANNER_HPP_
#define NAVYU_PLANNER__NAVYU_GLOBAL_PLANNER_HPP_

#include "navyu_planner/astar_planner.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

class NavyuGlobalPlanner : public rclcpp::Node
{
public:
  explicit NavyuGlobalPlanner(const rclcpp::NodeOptions & node_options);
  ~NavyuGlobalPlanner();

  bool plan(
    const geometry_msgs::msg::Pose start, const geometry_msgs::msg::Pose goal,
    std::vector<Node2D *> & path);

  void callback_costmap(const nav_msgs::msg::OccupancyGrid & msg);
  void callback_goal_pose(const geometry_msgs::msg::PoseStamped & msg);

  void publish_path(std::vector<Node2D *> path);

  void wait_for_costmap();

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

  std::shared_ptr<AstarPlanner> planner_;

  std::string map_frame_id_;

  bool costmap_initialized_{false};
};

#endif  // NAVYU_PLANNER__NAVYU_GLOBAL_PLANNER_HPP_

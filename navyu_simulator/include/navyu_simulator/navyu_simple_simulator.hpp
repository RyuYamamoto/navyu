#ifndef _NAVYU_SIMULATOR__NAVYU_SIMPLE_SIMULATOR_HPP_
#define _NAVYU_SIMULATOR__NAVYU_SIMPLE_SIMULATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <rclcpp/timer.hpp>

class NavyuSimpleSimulator : public rclcpp::Node
{
public:
  NavyuSimpleSimulator(const rclcpp::NodeOptions & node_options);
  ~NavyuSimpleSimulator();

  void callback_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr &msg);
  void callback_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr &msg);

  void motion();

  void process();

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr current_pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;

  geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg_;
};

#endif

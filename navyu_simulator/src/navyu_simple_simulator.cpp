#include "navyu_simulator/navyu_simple_simulator.hpp"

NavyuSimpleSimulator::NavyuSimpleSimulator(const rclcpp::NodeOptions & node_options) : Node("navyu_simple_simulator", node_options)
{
}

void NavyuSimpleSimulator::callback_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr &msg)
{
  cmd_vel_msg_ = msg;
}

void NavyuSimpleSimulator::callback_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr &msg)
{
  current_pose_->pose = msg->pose.pose;
  current_pose_->header = msg->header;
}

void NavyuSimpleSimulator::motion()
{
}

void NavyuSimpleSimulator::process()
{
}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(NavyuSimpleSimulator)

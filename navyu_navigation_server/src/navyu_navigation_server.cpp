#include "navyu_navigation_server/navyu_navigation_server.hpp"

NavyuNavigationServer::NavyuNavigationServer(const rclcpp::NodeOptions & node_options)
: Node("navyu_navigation_server", node_options)
{
  map_frame_ = declare_parameter<std::string>("map_frame");
  base_frame_ = declare_parameter<std::string>("base_frame");
  update_frequency_ = declare_parameter<double>("update_frequency");

  goal_pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 5,
    std::bind(&NavyuNavigationServer::callback_goal_pose, this, std::placeholders::_1));

  goal_client_ = create_client<navyu_msgs::srv::Goal>("goal");

  timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1.0 / update_frequency_ * 1000)),
    std::bind(&NavyuNavigationServer::update, this));
}

NavyuNavigationServer::~NavyuNavigationServer()
{
}

void NavyuNavigationServer::update()
{
  if (!get_robot_pose(current_pose_.pose)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Cant not get Robot Pose.");
    return;
  }

  if (is_planning_) {
    auto request = std::make_shared<navyu_msgs::srv::Goal::Request>();
    request->goal.pose = goal_pose_.pose;
    request->start.pose = current_pose_.pose;

    auto response = goal_client_->async_send_request(request);
  }
}

bool NavyuNavigationServer::get_robot_pose(geometry_msgs::msg::Pose & robot_pose)
{
  geometry_msgs::msg::TransformStamped robot_pose_frame;

  try {
    robot_pose_frame = tf_buffer_.lookupTransform(
      map_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Can not get Transform " << map_frame_ << " to " << base_frame_);
    return false;
  }

  robot_pose.position.x = robot_pose_frame.transform.translation.x;
  robot_pose.position.y = robot_pose_frame.transform.translation.y;
  robot_pose.position.z = robot_pose_frame.transform.translation.z;
  robot_pose.orientation = robot_pose_frame.transform.rotation;

  return true;
}
void NavyuNavigationServer::callback_goal_pose(const geometry_msgs::msg::PoseStamped msg)
{
  if (!get_robot_pose(current_pose_.pose)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Cant not get Robot Pose.");
    return;
  }
  is_planning_ = true;
  goal_pose_ = msg;

  auto request = std::make_shared<navyu_msgs::srv::Goal::Request>();
  request->goal.pose = goal_pose_.pose;
  request->start.pose = current_pose_.pose;

  auto response = goal_client_->async_send_request(request);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(NavyuNavigationServer)

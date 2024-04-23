#include "navyu_planner/navyu_global_planner.hpp"

NavyuGlobalPlanner::NavyuGlobalPlanner(const rclcpp::NodeOptions & node_options)
: Node("navyu_global_planner", node_options)
{
  map_frame_id_ = declare_parameter<std::string>("map_frame_id");

  declare_parameter<double>("lethal_cost_threshold", 45);

  goal_pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 5,
    std::bind(&NavyuGlobalPlanner::callback_goal_pose, this, std::placeholders::_1));
  costmap_subscriber_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "costmap", rclcpp::QoS(10).transient_local().reliable().keep_last(1),
    std::bind(&NavyuGlobalPlanner::callback_costmap, this, std::placeholders::_1));

  path_publisher_ = create_publisher<nav_msgs::msg::Path>(
    "path", rclcpp::QoS(10).transient_local().reliable().keep_last(1));
}

NavyuGlobalPlanner::~NavyuGlobalPlanner()
{
}

void NavyuGlobalPlanner::wait_for_costmap()
{
  rclcpp::WallRate rate(1);
  while (!costmap_initialized_) {
    RCLCPP_INFO_STREAM(get_logger(), "Wait For Costmap...");
    rate.sleep();
  }
}

bool NavyuGlobalPlanner::plan(
  const geometry_msgs::msg::Pose start, const geometry_msgs::msg::Pose goal,
  std::vector<Node2D *> & path)
{
  int start_x, start_y;
  int goal_x, goal_y;
  planner_->convert_map_to_grid(start.position.x, start.position.y, start_x, start_y);
  planner_->convert_map_to_grid(goal.position.x, goal.position.y, goal_x, goal_y);

  Node2D * start_node = new Node2D(start_x, start_y, 0.0, NULL);
  start_node->set_grid_index(planner_->get_grid_index(start_x, start_y));

  Node2D * goal_node = new Node2D(goal_x, goal_y, 0.0, NULL);
  goal_node->set_grid_index(planner_->get_grid_index(goal_x, goal_y));

  return planner_->plan(start_node, goal_node, path);
}

void NavyuGlobalPlanner::publish_path(std::vector<Node2D *> path)
{
  nav_msgs::msg::Path path_msgs;
  path_msgs.header.frame_id = map_frame_id_;
  path_msgs.header.stamp = now();

  for (auto & node : path) {
    geometry_msgs::msg::PoseStamped path_pose;
    path_pose.header = path_msgs.header;
    planner_->convert_grid_to_map(
      node->x_, node->y_, path_pose.pose.position.x, path_pose.pose.position.y);
    path_msgs.poses.emplace_back(path_pose);
  }

  path_publisher_->publish(path_msgs);
}

void NavyuGlobalPlanner::callback_goal_pose(const geometry_msgs::msg::PoseStamped & msg)
{
  if (planner_ == nullptr) {
    RCLCPP_ERROR_STREAM(get_logger(), "planner is not Initialized.");
    return;
  }

  geometry_msgs::msg::Pose start;
  start.position.x = 0.0;
  start.position.y = 0.0;

  std::vector<Node2D *> path;
  if (plan(start, msg.pose, path)) {
    publish_path(path);
  }
}

void NavyuGlobalPlanner::callback_costmap(const nav_msgs::msg::OccupancyGrid & msg)
{
  if (!costmap_initialized_) {
    costmap_initialized_ = true;

    int size_x = msg.info.width;
    int size_y = msg.info.height;
    double resolution = msg.info.resolution;
    double origin_x = msg.info.origin.position.x;
    double origin_y = msg.info.origin.position.y;

    // initialize global planner method
    planner_ = std::make_shared<AstarPlanner>(size_x, size_y, resolution, origin_x, origin_y);
    planner_->set_costmap(msg.data);

    double lethal_cost_threshold;
    get_parameter("lethal_cost_threshold", lethal_cost_threshold);
    planner_->set_lethal_cost_threshold(lethal_cost_threshold);

    return;
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(NavyuGlobalPlanner)

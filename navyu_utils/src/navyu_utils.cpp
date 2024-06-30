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

#include "navyu_utils/navyu_utils.hpp"

namespace navyu_utils
{

bool get_robot_pose(
  const std::string global_frame, const std::string robot_frame, tf2_ros::Buffer & tf_buffer,
  geometry_msgs::msg::Pose & robot_pose)
{
  static rclcpp::Logger logger = rclcpp::get_logger("get_robot_pose");

  geometry_msgs::msg::TransformStamped robot_pose_frame;

  try {
    robot_pose_frame = tf_buffer.lookupTransform(
      global_frame, robot_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(logger, "Can not get Transform " << global_frame << " to " << robot_frame);
    return false;
  }

  robot_pose.position.x = robot_pose_frame.transform.translation.x;
  robot_pose.position.y = robot_pose_frame.transform.translation.y;
  robot_pose.position.z = robot_pose_frame.transform.translation.z;
  robot_pose.orientation = robot_pose_frame.transform.rotation;

  return true;
}

double calculation_distance(const geometry_msgs::msg::Pose p1, const geometry_msgs::msg::Pose p2)
{
  const double dx = p1.position.x - p2.position.x;
  const double dy = p1.position.y - p2.position.y;
  return std::hypot(dx, dy);
}

double normalized_radian(const double radian)
{
  double normalized_radian = radian;
  if (M_PI < radian)
    normalized_radian -= 2.0 * M_PI;
  else if (radian < -M_PI)
    normalized_radian += 2.0 * M_PI;
  return normalized_radian;
}

}  // namespace navyu_utils

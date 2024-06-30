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

#ifndef NAVYU_UTILS__QUATERNION_UTILS_HPP_
#define NAVYU_UTILS__QUATERNION_UTILS_HPP_

#include <Eigen/Core>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>

namespace quaternion_utils
{

geometry_msgs::msg::Quaternion convert_euler_to_quaternion(const geometry_msgs::msg::Vector3 euler);

geometry_msgs::msg::Vector3 convert_quaternion_to_euler(
  const geometry_msgs::msg::Quaternion quaternion);

geometry_msgs::msg::Quaternion rotation(
  const geometry_msgs::msg::Quaternion quat1, const geometry_msgs::msg::Quaternion quat2);

Eigen::Matrix3f get_rotation_matrix_from_euler(const geometry_msgs::msg::Vector3 euler);

}  // namespace quaternion_utils

#endif

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

#include "navyu_utils/quaternion_utils.hpp"

namespace quaternion_utils
{

geometry_msgs::msg::Quaternion convert_euler_to_quaternion(const geometry_msgs::msg::Vector3 euler)
{
  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(euler.x, euler.y, euler.z);

  geometry_msgs::msg::Quaternion quaternion = tf2::toMsg(tf2_quat);

  return quaternion;
}

geometry_msgs::msg::Vector3 convert_quaternion_to_euler(
  const geometry_msgs::msg::Quaternion quaternion)
{
  geometry_msgs::msg::Vector3 euler;

  tf2::Quaternion quat(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
  tf2::Matrix3x3 mat(quat);
  mat.getRPY(euler.x, euler.y, euler.z);

  return euler;
}

geometry_msgs::msg::Quaternion rotation(
  const geometry_msgs::msg::Quaternion quat1, const geometry_msgs::msg::Quaternion quat2)
{
  geometry_msgs::msg::Quaternion result;

  result.x = quat1.w * quat2.x - quat1.z * quat2.y + quat1.y * quat2.z + quat1.x * quat2.w;
  result.y = quat1.z * quat2.x + quat1.w * quat2.y - quat1.x * quat2.z + quat1.y * quat2.w;
  result.z = -quat1.y * quat2.x + quat1.x * quat2.y + quat1.w * quat2.z + quat1.z * quat2.w;
  result.w = -quat1.z * quat2.x - quat1.y * quat2.y + -quat1.z * quat2.z + quat1.w * quat2.w;

  return result;
}

Eigen::Matrix3f get_rotation_matrix_from_euler(const geometry_msgs::msg::Vector3 euler)
{
  Eigen::Quaternionf rotation_matrix = Eigen::AngleAxisf(euler.x, Eigen::Vector3f::UnitX()) *
                                       Eigen::AngleAxisf(euler.y, Eigen::Vector3f::UnitY()) *
                                       Eigen::AngleAxisf(euler.z, Eigen::Vector3f::UnitZ());

  return rotation_matrix.toRotationMatrix();
}

Eigen::Matrix3f get_rotation_matrix_from_quaternion(const geometry_msgs::msg::Quaternion quaternion)
{
  Eigen::Quaternionf eigen_quat;
  eigen_quat.x() = quaternion.x;
  eigen_quat.y() = quaternion.y;
  eigen_quat.z() = quaternion.z;
  eigen_quat.w() = quaternion.w;

  return eigen_quat.toRotationMatrix();
}

}  // namespace quaternion_utils

#include "pose.hpp"

#include <algorithm> // max
#include <cmath>
#include <stdexcept>

const float Pose::kDefaultMaxJointStep_deg = 10.0;
const float Pose::kInvNumJoints = 0.166666;

Pose::Pose(float j1_deg, float j2_deg, float j3_deg, float j4_deg, float j5_deg, float j6_deg)
    : J1_deg(j1_deg), J2_deg(j2_deg), J3_deg(j3_deg), J4_deg(j4_deg), J5_deg(j5_deg), J6_deg(j6_deg)
{
  joint_values_deg = {J1_deg, J2_deg, J3_deg, J4_deg, J5_deg, J6_deg};
}

Pose::Pose(const std::vector<float>& pose_joint_values_deg)
{
  if (pose_joint_values_deg.size() != kNumJoints) {
    throw std::runtime_error("Pose: Invalid number of joint values provided in vector ("
                            + std::to_string(joint_values_deg.size()) + "). Should be "
                            + std::to_string(kNumJoints) + ".");
  }
  J1_deg = pose_joint_values_deg[0];
  J2_deg = pose_joint_values_deg[1];
  J3_deg = pose_joint_values_deg[2];
  J4_deg = pose_joint_values_deg[3];
  J5_deg = pose_joint_values_deg[4];
  J6_deg = pose_joint_values_deg[5];

  joint_values_deg = pose_joint_values_deg;
}

void Pose::Clear()
{
  J1_deg = 0.0;
  J2_deg = 0.0;
  J3_deg = 0.0;
  J4_deg = 0.0;
  J5_deg = 0.0;
  J6_deg = 0.0;
}

float Pose::JointDistTo(const Pose& pose) const
{
  return std::abs(pose.J1_deg - J1_deg)
       + std::abs(pose.J2_deg - J2_deg)
       + std::abs(pose.J3_deg - J3_deg)
       + std::abs(pose.J4_deg - J4_deg)
       + std::abs(pose.J5_deg - J5_deg)
       + std::abs(pose.J6_deg - J6_deg);
}

float Pose::MaxJointDistTo(const Pose& pose) const
{
  return std::max({std::abs(J1_deg - pose.J1_deg),
                   std::abs(J2_deg - pose.J2_deg),
                   std::abs(J3_deg - pose.J3_deg),
                   std::abs(J4_deg - pose.J4_deg),
                   std::abs(J5_deg - pose.J5_deg),
                   std::abs(J6_deg - pose.J6_deg)});
}


bool Pose::IsValid(const ArmDimensions& arm_dimensions) const
{
  if (arm_dimensions.joint_limits_deg.size() != kNumJoints) {
    throw std::runtime_error("Pose: Invalid number of joint ranges provided. Should be "
                            + std::to_string(kNumJoints) + ".");
  }
  for (int j = 0; j < kNumJoints; j++) {
    if (joint_values_deg[j] < arm_dimensions.joint_limits_deg[j].first
     || joint_values_deg[j] > arm_dimensions.joint_limits_deg[j].second) {
      return false;
    }
  }
  return true;
}

std::vector<Pose> Pose::PoseEdgeTo(const Pose& pose, float max_joint_step_deg) const
{
  std::vector<Pose> pose_edge;
  float max_angle_diff_deg =
    std::max({std::abs(pose.J1_deg - J1_deg),
              std::abs(pose.J2_deg - J2_deg),
              std::abs(pose.J3_deg - J3_deg),
              std::abs(pose.J4_deg - J4_deg),
              std::abs(pose.J5_deg - J5_deg),
              std::abs(pose.J6_deg - J6_deg)});

  int num_steps = std::ceil(max_angle_diff_deg / max_joint_step_deg);
  float inv_num_steps = 1.0 / num_steps;
  float J1_step_deg = (pose.J1_deg - J1_deg) * inv_num_steps;
  float J2_step_deg = (pose.J2_deg - J2_deg) * inv_num_steps;
  float J3_step_deg = (pose.J3_deg - J3_deg) * inv_num_steps;
  float J4_step_deg = (pose.J4_deg - J4_deg) * inv_num_steps;
  float J5_step_deg = (pose.J5_deg - J5_deg) * inv_num_steps;
  float J6_step_deg = (pose.J6_deg - J6_deg) * inv_num_steps;

  for (int i = 0; i <= num_steps; i++) {
    pose_edge.push_back(Pose(
      J1_deg + i * J1_step_deg,
      J2_deg + i * J2_step_deg,
      J3_deg + i * J3_step_deg,
      J4_deg + i * J4_step_deg,
      J5_deg + i * J5_step_deg,
      J6_deg + i * J6_step_deg
    ));
  }
  return pose_edge;
}

bool Pose::operator==(const Pose& pose) const
{
  float epsilon = 0.001;
  return std::abs(pose.J1_deg - J1_deg) < epsilon
      && std::abs(pose.J2_deg - J2_deg) < epsilon
      && std::abs(pose.J3_deg - J3_deg) < epsilon
      && std::abs(pose.J4_deg - J4_deg) < epsilon
      && std::abs(pose.J5_deg - J5_deg) < epsilon
      && std::abs(pose.J6_deg - J6_deg) < epsilon;
}

bool Pose::operator!=(const Pose& pose) const
{
  return !(operator==(pose));
}
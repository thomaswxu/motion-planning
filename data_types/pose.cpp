#include "pose.hpp"

#include <cmath>

const float Pose::kInvNumJoints = 0.166666;

Pose::Pose(float j1, float j2, float j3, float j4, float j5, float j6)
    : J1(j1), J2(j2), J3(j3), J4(j4), J5(j5), J6(j6)    
{}

float Pose::JointDistTo(const Pose& pose) const
{
  return std::abs(pose.J1 - J1)
       + std::abs(pose.J2 - J2)
       + std::abs(pose.J3 - J3)
       + std::abs(pose.J4 - J4)
       + std::abs(pose.J5 - J5)
       + std::abs(pose.J6 - J6);
}

bool Pose::operator==(const Pose& pose) const
{
  float epsilon = 0.001;
  return std::abs(pose.J1 - J1) < epsilon
      && std::abs(pose.J2 - J2) < epsilon
      && std::abs(pose.J3 - J3) < epsilon
      && std::abs(pose.J4 - J4) < epsilon
      && std::abs(pose.J5 - J5) < epsilon
      && std::abs(pose.J6 - J6) < epsilon;
}

bool Pose::operator!=(const Pose& pose) const
{
  return !(operator==(pose));
}
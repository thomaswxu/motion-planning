/**
 * Class to represent the joint pose of the robot arm. Assumes standard 6DOF arm, fixed base.
 * Joint indices increase starting from the base.
 */
#pragma once

#include <vector>

#include "arm_dimensions.hpp"

class Pose
{
public:
  Pose(float j1_deg, float j2_deg, float j3_deg, float j4_deg, float j5_deg, float j6_deg);
  Pose(const std::vector<float>& pose_joint_values_deg);

  /** Reset all joint values.*/
  void Clear();

  /** Return the sum of individual joint distances to another pose.*/
  float JointDistTo(const Pose& pose) const;

  /** Check if the stored joint values are valid given some joint ranges.*/
  bool IsValid(const ArmDimensions& arm_dimensions) const;

  /** Get a vector of intermediate, evenly spaced poses from this one to a given one, inclusive.*/
  std::vector<Pose> PoseEdgeTo(const Pose& pose, float max_joint_step_deg=kDefaultMaxJointStep_deg) const;

  bool operator==(const Pose& pose) const;
  bool operator!=(const Pose& pose) const;

  float J1_deg;
  float J2_deg;
  float J3_deg;
  float J4_deg;
  float J5_deg;
  float J6_deg;

  std::vector<float> joint_values_deg; // Additional vector form for convenience.

  static const float kDefaultMaxJointStep_deg; // For computing pose edges
  static const float kInvNumJoints; // Precompute to avoid slow divisions
  static const int kNumJoints = 6;
};
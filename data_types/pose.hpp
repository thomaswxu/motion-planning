/**
 * Class to represent the joint pose of the robot arm. Assumes standard 6DOF arm, fixed base.
 * Joint indices increase starting from the base.
 */
#pragma once

#include <vector>

class Pose
{
public:
  Pose(float j1_deg, float j2_deg, float j3_deg, float j4_deg, float j5_deg, float j6_deg);

  /** Return the sum of individual joint distances to another pose.*/
  float JointDistTo(const Pose& pose) const;

  /** Get a vector of intermediate, evenly spaced poses from this one to a given one, inclusive.*/
  std::vector<Pose> PoseEdgeTo(const Pose& pose, float max_joint_step_deg=kDefaultMaxJointStep_deg) const;

  bool operator==(const Pose& pose) const;
  bool operator!=(const Pose& pose) const;

  const float J1_deg;
  const float J2_deg;
  const float J3_deg;
  const float J4_deg;
  const float J5_deg;
  const float J6_deg;

  static const float kDefaultMaxJointStep_deg; // For computing pose edges
  static const float kInvNumJoints; // Precompute to avoid slow divisions
};
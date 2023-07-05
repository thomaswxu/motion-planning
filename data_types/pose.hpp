/**
 * Class to represent the joint pose of the robot arm. Assumes standard 6DOF arm, fixed base.
 * Joint indices increase starting
 */
#pragma once

class Pose
{
public:
  Pose(float j1, float j2, float j3, float j4, float j5, float j6);

  /** Return the sum of individual joint distances to another pose.*/
  float JointDistTo(const Pose& pose) const;

  bool operator==(const Pose& pose) const;
  bool operator!=(const Pose& pose) const;

  const float J1;
  const float J2;
  const float J3;
  const float J4;
  const float J5;
  const float J6;

  static const float kInvNumJoints; // Precompute to avoid slow divisions
};
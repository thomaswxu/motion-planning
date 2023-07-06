/**
 * Class to represent an arm pose in terms of the Cartesian positions of each
 * of its joints.
 */
#pragma once

#include <vector>

#include "arm_dimensions.hpp"
#include "edge.hpp"
#include "pose.hpp"
#include "vec3.hpp"

class PosePoints
{
public:

private:
  const Pose pose_;
  // const ArmDimensions arm_dimensions_;

  const std::vector<Vec3> joint_points_; // Each joint XYZ, in ascending order from arm base
  const std::vector<Edge> joint_edges_;  // Each link as an edge, in ascending order from arm base

  Edge EE_edge_;
  float EE_length_mm_;
  float EE_radius_mm_;
};
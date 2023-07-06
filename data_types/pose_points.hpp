/**
 * Class to represent an arm pose in terms of the Cartesian positions of each
 * of its joints.
 */
#pragma once

#include <cmath>
#include <vector>

#include "arm_dimensions.hpp"
#include "edge.hpp"
#include "pose.hpp"
#include "vec3.hpp"

class PosePoints
{
public:
  PosePoints(const Pose& pose, const ArmDimensions& arm_dimensions);

  inline std::vector<Vec3> arm_points() const { return arm_points_; }
  inline std::vector<Edge> arm_edges() const { return arm_edges_; }

  static inline float Deg2Rad(float deg) { return deg / 180.0 * M_PI; };

private:
  void CalcArmPoints();
  void CalcArmEdges();

  const Pose pose_;
  const ArmDimensions arm_dimensions_;

  std::vector<Vec3> arm_points_; // Each joint XYZ and EE tip XYZ, in ascending order from arm base
  std::vector<Edge> arm_edges_;  // Each link as an edge (incl. EE), in ascending order from arm base
};
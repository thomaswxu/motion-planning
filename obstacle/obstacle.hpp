/**
 * Class for obstacles that are planned around. For simplicity, model all obstacles as axis-aligned bounding boxes.
 */
#pragma once

#include <vector>

#include "arm_dimensions.hpp"
#include "edge.hpp"
#include "pose.hpp"
#include "pose_points.hpp"
#include "vec3.hpp"

class Obstacle
{
public:
  Obstacle(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z);

  /** Detect whether a given XYZ point is inside or touches the obstacle.*/
  bool PointIsInside(const Vec3& point) const;

  /** 
   * Detect whether a given edge is inside or touches the obstacle.
   * @param edge Edge to check.
   * @param collision_check_step_mm Step distance to use for discretized collision check, millimeters.
   */
  bool EdgeIsInside(const Edge& edge, int collision_check_step_mm=50) const;

  /** Detect whether a given arm pose is inside or touches the obstacle.*/
  bool PoseIsInside(const Pose& pose, const ArmDimensions& arm_dims) const;
  /** Overload that makes use of precomputed pose points and inverse vectors.*/
  bool PoseIsInside(const PosePoints& pose_points, const std::vector<Vec3>& arm_edge_inverse_vectors,
                    const ArmDimensions& arm_dims) const;

  /** Detect whether the a pose path (assuming uniform/constant joint velocities) touches the obstacle.*/
  bool PoseEdgeIsInside(const std::vector<Pose>& poses, const std::vector<PosePoints>& all_pose_points,
                        const std::vector<std::vector<Vec3>>& all_poses_inverse_vectors,
                        const ArmDimensions& arm_dims) const;

  /** Get the distance to a given point.*/
  float DistToPoint(const Vec3& point) const;

  /** Get the distance to a given edge.*/
  float DistTOEdge(const Edge& edge) const;

  static const float kJointStep_deg; // Could be a config parameter or passed in as an argument, if desired.

private:
  const float min_x_;
  const float max_x_;
  const float min_y_;
  const float max_y_;
  const float min_z_;
  const float max_z_;
  const Vec3 min_; // [min_x_, min_y_, min_z_]
  const Vec3 max_; // [max_x_, max_y_, max_z_]
};
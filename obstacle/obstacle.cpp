#include "obstacle.hpp"

#include <algorithm> // clamp
#include <limits>
#include <stdexcept>

Obstacle::Obstacle(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z)
    : min_x_(min_x), max_x_(max_x), min_y_(min_y), max_y_(max_y), min_z_(min_z), max_z_(max_z)
{
  if (max_x_ < min_x_) {
    throw std::runtime_error("Obstacle: Invalid dimensions; max x < min_x.");
  }
  if (max_y_ < min_y_) {
    throw std::runtime_error("Obstacle: Invalid dimensions; max y < min_y.");
  }
  if (max_z_ < min_z_) {
    throw std::runtime_error("Obstacle: Invalid dimensions; max z < min_z.");
  }
}

bool Obstacle::PointIsInside(const Vec3& point) const
{
  return point.x() >= min_x_ && point.x() <= max_x_
      && point.y() >= min_y_ && point.y() <= max_y_
      && point.z() >= min_z_ && point.z() <= max_z_;
}

bool Obstacle::EdgeIsInside(const Edge& edge, int collision_check_step_mm) const
{
  if (PointIsInside(edge.start_point()) || PointIsInside(edge.end_point())) {
    return true;
  }
  int check_dist_mm = 0;
  while (check_dist_mm <= edge.length()) {
    if (PointIsInside(edge.IntermediatePoint(check_dist_mm))) {
      return true;
    }
    check_dist_mm += collision_check_step_mm;
  }
  return false;
}

bool Obstacle::PoseIsInside(const Pose& pose, const ArmDimensions& arm_dims) const
{
  PosePoints pose_points(pose, arm_dims); // Note: better to precompute this if there are many obstacles to check.
  std::vector<Vec3> edge_inv_vecs = pose_points.ArmEdgeInverseVectors(); // Also best to precompute these.
  return PoseIsInside(pose_points, edge_inv_vecs, arm_dims);
}

bool Obstacle::PoseIsInside(const PosePoints& pose_points, const std::vector<Vec3>& arm_edge_inverse_vectors,
                            const ArmDimensions& arm_dims) const
{
  if (pose_points.arm_edges().size() != arm_edge_inverse_vectors.size()) {
    throw std::runtime_error("Obstacle: Mismatched sizes of pose points and arm edge inverse vectors.");
  }

  int arm_edge_num = 0;
  for (const Edge& arm_edge : pose_points.arm_edges()) {
    // Using raytracing slab method; see this for details: https://tavianator.com/2022/ray_box_boundary.html
    float t_min = -std::numeric_limits<float>::infinity();
    float t_max =  std::numeric_limits<float>::infinity();
    Vec3 edge_inv_vec = arm_edge_inverse_vectors[arm_edge_num];

    for (int dimension = 0; dimension < 3; dimension++) {
      float t1 =
        (min_.Component(dimension) - arm_edge.start_point().Component(dimension)) * edge_inv_vec.Component(dimension);
      float t2 =
        (max_.Component(dimension) - arm_edge.start_point().Component(dimension)) * edge_inv_vec.Component(dimension);
      
      t_min = std::min(std::max(t1, t_min), std::max(t2, t_min));
      t_max = std::max(std::min(t1, t_max), std::min(t2, t_max));
    }
    // Clamp coefficients for use with non-normalized edge vectors
    t_min = std::clamp(t_min, float(0), float(1));
    t_max = std::clamp(t_max, float(0), float(1));

    // Check edge collision based on shortest dist to box (collides if less than arm link radius)
    Vec3 closest_pt_on_box(
      std::clamp(float(0.5 * (arm_edge.start_point().x() + (arm_edge.vec().x() * t_min)
                            + arm_edge.start_point().x() + (arm_edge.vec().x() * t_max))), min_x_, max_x_),
      std::clamp(float(0.5 * (arm_edge.start_point().y() + (arm_edge.vec().y() * t_min)
                            + arm_edge.start_point().y() + (arm_edge.vec().y() * t_max))), min_y_, max_y_),
      std::clamp(float(0.5 * (arm_edge.start_point().z() + (arm_edge.vec().z() * t_min)
                            + arm_edge.start_point().z() + (arm_edge.vec().z() * t_max))), min_z_, max_z_)
    );
    // Get t coefficient value for closest point on edge
    float t_close = std::clamp( // Clamp to constrain to edge, not full line
      (closest_pt_on_box - arm_edge.start_point()).Dot(arm_edge.vec()) / arm_edge.vec().NormSq(), float(0), float(1));
    Vec3 closest_pt_on_edge = arm_edge.start_point() + arm_edge.vec().Scaled(t_close);

    if (closest_pt_on_edge.DistTo(closest_pt_on_box) < arm_dims.L_radius_mm) {
      return true;
    }
    arm_edge_num++;
  }
  return false;
}

bool Obstacle::PoseEdgeIsInside(const std::vector<Pose>& poses, const std::vector<PosePoints>& all_pose_points,
                                const std::vector<std::vector<Vec3>>& all_poses_inverse_vectors,
                                const ArmDimensions& arm_dims) const
{
  if (all_pose_points.size() != all_poses_inverse_vectors.size()) {
    throw std::runtime_error("Obstacle: Mismatched number of pose points and sets of arm edge inverse vectors.");
  }
  for (int i = 0; i < all_pose_points.size(); i++) {
    if (PoseIsInside(all_pose_points[i], all_poses_inverse_vectors[i], arm_dims)) {
      return true;
    }
  }
  return false;
}
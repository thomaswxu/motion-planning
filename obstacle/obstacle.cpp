#include "obstacle.hpp"

#include <stdexcept>

const float Obstacle::kJointStep_deg = 10.0;

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
  return false;
}

bool Obstacle::PoseIsInside(const Pose& pose, const ArmDimensions& arm_dims) const
{
  return false;
}

bool Obstacle::PoseEdgeIsInside(const Pose& pose1, const Pose& pose2, const ArmDimensions& arm_dims) const
{
  return false;
}
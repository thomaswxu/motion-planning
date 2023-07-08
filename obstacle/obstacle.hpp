/**
 * Class for obstacles that are planned around. For simplicity, model all obstacles as axis-aligned bounding boxes.
 * Units should be in millimeters unless specified otherwise.
 */
#pragma once

#include <string>
#include <vector>

#include "arm_dimensions.hpp"
#include "edge.hpp"
#include "pose.hpp"
#include "pose_points.hpp"
#include "vec3.hpp"

class Obstacle
{
public:
  Obstacle(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z, const std::string& name="");

  /** Constructor that reads obstacle info from a provided file, and the obstacle name in the file.*/
  Obstacle(const std::string& config_file, const std::string& name);

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

  /** Convenience function to read all obstacles in a configuration file.*/
  static std::vector<Obstacle> ObstaclesFromConfigFile(const std::string& config_file);

  inline float min_x() const { return min_x_; }
  inline float max_x() const { return max_x_; }
  inline float min_y() const { return min_y_; }
  inline float max_y() const { return max_y_; }
  inline float min_z() const { return min_z_; }
  inline float max_z() const { return max_z_; }
  inline Vec3 min() const { return min_; }
  inline Vec3 max() const { return max_; }
  inline std::string name() const { return name_; }

private:

  /** Throw an exception if the obstacle dimensions are invalid or don't make sense.*/
  void CheckDimensions() const;

  float min_x_;
  float max_x_;
  float min_y_;
  float max_y_;
  float min_z_;
  float max_z_;
  Vec3 min_; // [min_x_, min_y_, min_z_]
  Vec3 max_; // [max_x_, max_y_, max_z_]

  std::string name_;
};
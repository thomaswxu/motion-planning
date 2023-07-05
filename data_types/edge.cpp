#include "edge.hpp"

#include <iostream>

Edge::Edge(const Vec3& start_point, const Vec3& end_point)
    : start_point_(start_point),
      end_point_(end_point),
      vec_(end_point - start_point),
      length_((end_point - start_point).Norm())
{}

Vec3 Edge::ExtendedStart(float dist) const
{
  Vec3 normalized_vec = vec_.Normalized();
  Vec3 extended_start = start_point_;
  return Vec3(start_point_.x() - dist * normalized_vec.x(),
              start_point_.y() - dist * normalized_vec.y(),
              start_point_.z() - dist * normalized_vec.z());
}

Vec3 Edge::ExtendedEnd(float dist) const
{
  Vec3 normalized_vec = vec_.Normalized();
  Vec3 extended_end = end_point_;
  return Vec3(end_point_.x() + dist * normalized_vec.x(),
              end_point_.y() + dist * normalized_vec.y(),
              end_point_.z() + dist * normalized_vec.z());
}

Vec3 Edge::IntermediatePoint(float dist) const
{
  return ExtendedStart(-dist);
}

float Edge::DistTo(const Edge& edge) const
{
  if (vec_.IsParallelTo(edge.vec())) {
    return DistToParallelEdge(edge);
  }
  else {
    return DistToNonParallelEdge(edge);
  }
}

float Edge::DistToParallelEdge(const Edge& edge) const
{
  // Parallel: check if edges/line segments "overlap"

  // Unscaled projection distances
  float proj_dist_start = vec().Dot(edge.start_point() - start_point_);
  float proj_dist_end = vec().Dot(edge.end_point() - start_point_);

  bool other_edge_comes_before = proj_dist_start <= 0 && proj_dist_end <= 0;
  bool other_edge_comes_after = proj_dist_start >= length_ * length_ && proj_dist_end >= length_ * length_;

  if (other_edge_comes_before) {
    // No overlap and other edge is "before" this one
    if (std::abs(proj_dist_start) > std::abs(proj_dist_end)) {
      return (edge.end_point() - start_point_).Norm();
    }
    return (edge.start_point() - start_point_).Norm();
  }
  else if (other_edge_comes_after) {
    // No overlap and other edge is "before" this one
    if (std::abs(proj_dist_start) > std::abs(proj_dist_end)) {
      return (edge.end_point() - end_point_).Norm();
    }
    return (edge.start_point() - end_point_).Norm();
  }
  else {
    // Edges overlap: use distance between parallel lines formula
    return vec_.Cross(edge.start_point() - start_point_).Norm() / vec_.Norm();
  }
}

float Edge::DistToNonParallelEdge(const Edge& edge) const
{
  Vec3 perpendicular = vec_.Cross(edge.vec());

  // Get closest points without considering edge length (standard math method)
  // ("t" is scalar in standard line equation)
  float t1 = edge.vec().Cross(perpendicular).Dot(edge.start_point() - start_point_) / perpendicular.NormSq();
  float t2 = vec_.Cross(perpendicular).Dot(edge.start_point() - start_point_) / perpendicular.NormSq();
  Vec3 pt1 = start_point_ + vec_.Scaled(t1);
  Vec3 pt2 = edge.start_point() + edge.vec().Scaled(t2);

  // Clamp closest points to edges
  bool clamped1;
  bool clamped2;

  // Clamp this edge
  if (t1 > length_ * length_) {
    pt1 = end_point_;
    clamped1 = true;
  }
  else if (t1 < 0) {
    pt1 = start_point_;
    clamped1 = true;
  }
  // Clamp other edge
  if (t2 > edge.length() * edge.length()) {
    pt2 = edge.end_point();
    clamped2 = true;
  }
  else if (t2 < 0) {
    pt2 = edge.start_point();
    clamped2 = true;
  }

  // Recompute closest points after clamping
  if (clamped1) {
    float unscaled_projection_dist = edge.vec().Dot(pt1 - edge.start_point());
    if (unscaled_projection_dist < 0) {
      pt2 = edge.start_point();
    }
    else if (unscaled_projection_dist > edge.length() * edge.length()) {
      pt2 = edge.end_point();
    }
    else {
      pt2 = edge.start_point() + edge.vec().Normalized().Scaled(unscaled_projection_dist / edge.length());
    }
  }
  if (clamped2) {
    float unscaled_projection_dist = vec_.Dot(pt2 - start_point_);
    if (unscaled_projection_dist < 0) {
      pt1 = start_point_;
    }
    else if (unscaled_projection_dist > length_ * length_) {
      pt1 = end_point_;
    }
    else {
      pt1 = start_point_ + vec_.Normalized().Scaled(unscaled_projection_dist / length_);
    }
  }
  return (pt2 - pt1).Norm();
}
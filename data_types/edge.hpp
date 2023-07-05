/**
 * Simple class to represent a 3D line segment.
 */
#pragma once

#include "vec3.hpp"

class Edge
{
public:
  Edge(const Vec3& start_point, const Vec3& end_point);
  
  /** Return a point extended beyond the start point (extending the edge).*/
  Vec3 ExtendedStart(float dist) const;

  /** Return a point extended from the end point.*/
  Vec3 ExtendedEnd(float dist) const;

  /**
   * Get an intermediate (or extrapolated) point on the edge.
   * @param dist Distance from start point.
   */
  Vec3 IntermediatePoint(float dist) const;

  /** Return the shortest distance to another edge.*/
  float DistTo(const Edge& edge) const;

  inline Vec3 start_point() const { return start_point_; }
  inline Vec3 end_point() const { return end_point_; }
  inline float length() const { return length_; }
  inline Vec3 vec() const { return vec_; }

private:

  /** Get the shortest distance to another edge (assumed parallel).*/
  float DistToParallelEdge(const Edge& edge) const;

  /** Get the shortest distance to another edge (assumed not parallel).*/
  float DistToNonParallelEdge(const Edge& edge) const;

  const Vec3 start_point_;
  const Vec3 end_point_;
  const float length_;
  const Vec3 vec_; // Vector from start to end
};
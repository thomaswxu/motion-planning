/**
 * Simple class to represent a 3D line segment.
 */
#pragma once

#include "vec3.hpp"

class Edge
{
public:
  Edge() {}
  Edge(const Vec3& start_point, const Vec3& end_point);
  
private:
  Vec3 start_point_;
  Vec3 end_point_;
  float length_;
  Vec3 vec_;
};
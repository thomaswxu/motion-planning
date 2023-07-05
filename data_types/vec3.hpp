/**
 * Simple class to represent a 3D vector/point. Unitless.
 */
#pragma once

#include <cmath>

class Vec3
{
public:
  Vec3() {}
  Vec3(float x, float y, float z);

  /** Get the value of a single component.*/
  float Component(int dimension) const;

  /** Get the norm/magnitude.*/
  inline float Norm() const { return std::pow(x_*x_ + y_*y_ + z_*z_, 0.5); }

  /** Get the squared norm/magnitude.*/
  inline float NormSq() const { return x_*x_ + y_*y_ + z_*z_; }

  /** Get the distance to another vector.*/
  float DistTo(const Vec3& vec) const;

  /** Check if parallel to another vector.*/
  bool IsParallelTo(const Vec3& vec) const;

  /** Get the dot product with another vector.*/
  inline float Dot(const Vec3& vec) const { return (x_ * vec.x()) + (y_ * vec.y()) + (z_ * vec.z()); }

  /** Get the cross product with another vector.*/
  Vec3 Cross(const Vec3& vec) const;

  /** Get a normalized version of this vector.*/
  Vec3 Normalized() const;

  /** Get a scaled version of this vector.*/
  Vec3 Scaled(float scale) const;

  /** Intepreting as a point, move this point in a given direction by some scale factor.*/
  Vec3 Moved(const Vec3& direction, float factor) const;

  /** Get the inverse (element-wise reciprocal) of the vector.*/
  Vec3 Inverse() const;

  inline float x() const { return x_; }
  inline float y() const { return y_; }
  inline float z() const { return z_; }

  bool operator==(const Vec3& vec) const;
  bool operator!=(const Vec3& vec) const;
  Vec3 operator+(const Vec3& rhs) const;
  Vec3 operator-(const Vec3& rhs) const;

private:
  float x_;
  float y_;
  float z_;
};
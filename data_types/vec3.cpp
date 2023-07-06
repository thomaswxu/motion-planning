#include "vec3.hpp"

#include <stdexcept>

Vec3::Vec3(float x, float y, float z)
    : x_(x), y_(y), z_(z)
{}

float Vec3::Component(int dimension) const
{
  if (dimension < 0 || dimension > 2) {
    throw std::runtime_error("Vec3: Invalid dimension requested. Valid dimensions: [0, 2]");
  }
  if (dimension == 0) { return x_; }
  if (dimension == 1) { return y_; }
  if (dimension == 2) { return z_; }
}

float Vec3::DistTo(const Vec3& vec) const
{
  float x_dist = vec.x() - x_;
  float y_dist = vec.y() - y_;
  float z_dist = vec.z() - z_;
  return std::pow(x_dist*x_dist + y_dist*y_dist + z_dist*z_dist, 0.5);
}

bool Vec3::IsParallelTo(const Vec3& vec) const
{
  return Cross(vec) == Vec3(0, 0, 0);
}

Vec3 Vec3::Cross(const Vec3& vec) const
{
  return Vec3(y_*vec.z() - z_*vec.y(),
              z_*vec.x() - x_*vec.z(),
              x_*vec.y() - y_*vec.x());
}

Vec3 Vec3::Normalized() const
{
  float norm = Norm();
  if (norm == 0) {
    return Vec3(0, 0, 0);
  }
  return Vec3(x_ / norm, y_ / norm, z_ / norm);
}

Vec3 Vec3::Scaled(float scale) const
{
  return Vec3(x_ * scale, y_ * scale, z_ * scale);
}

Vec3 Vec3::Moved(const Vec3& direction, float factor) const
{
  return Vec3(x_ + factor * direction.x(),
              y_ + factor * direction.y(),
              z_ + factor * direction.z());
}

Vec3 Vec3::Inverse() const
{
  return Vec3(1.0 / x_, 1.0 / y_, 1.0 / z_);
}

bool Vec3::operator==(const Vec3& vec) const
{
  float epsilon = 0.001;
  return std::abs(x_ - vec.x()) < epsilon &&
         std::abs(y_ - vec.y()) < epsilon &&
         std::abs(z_ - vec.z()) < epsilon;
}

bool Vec3::operator!=(const Vec3& vec) const
{
  return !(this->operator==(vec));
}

Vec3 Vec3::operator+(const Vec3& rhs) const
{
  return Vec3(x_ + rhs.x(), y_ + rhs.y(), z_ + rhs.z());
}

Vec3 Vec3::operator-(const Vec3& rhs) const
{
  return Vec3(x_ - rhs.x(), y_ - rhs.y(), z_ - rhs.z());
}

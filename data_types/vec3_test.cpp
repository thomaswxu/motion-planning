#include "vec3.hpp"

#include <cmath>

#include <catch2/catch_test_macros.hpp>


TEST_CASE("Operators work", "[operators]")
{
  Vec3 v1(0.7, 0.16, -0.45);
  SECTION("Equality") {
    REQUIRE(v1 == Vec3(0.7, 0.16, -0.45));
    REQUIRE(v1 != Vec3(0.71, 0.16, -0.45));
    REQUIRE(v1 != Vec3(0.7, 0.165, -0.45));
    REQUIRE(v1 != Vec3(0, 0, 0));
    REQUIRE(v1 != Vec3(1, 2, 3));
  }
  SECTION("Addition") {
    REQUIRE(Vec3(1, 2, 3) + Vec3(1, 2, 3) == Vec3(2, 4, 6));
    REQUIRE(Vec3(1, 2, 3) + Vec3(-6, 2, 10) == Vec3(-5, 4, 13));
  }
  SECTION("Subtraction") {
    REQUIRE(Vec3(1, 2, 3) - Vec3(1, 2, 3) == Vec3(0, 0, 0));
    REQUIRE(Vec3(3, 3, 3) - Vec3(1, 2, 3) == Vec3(2, 1, 0));
  }
}

TEST_CASE("Distance calculations work", "[distance]")
{
  float epsilon = 0.0001;
  REQUIRE(Vec3(0, 0, 0).DistTo(Vec3(1, 0, 0)) - 1 < epsilon);
  REQUIRE(Vec3(0, 0, 0).DistTo(Vec3(0, 0, 0)) - 0 < epsilon);
  REQUIRE(Vec3(1, 2, 3).DistTo(Vec3(10, 3, -6)) - 12.7671 < epsilon);
}

TEST_CASE("Inverse calculations work", "[inverse]")
{
  REQUIRE(Vec3(1, 1, 2).Inverse() == Vec3(1, 1, 0.5));
  Vec3 inf_vec = Vec3(0, 0, 0).Inverse();
  REQUIRE(std::isinf(inf_vec.x()));
  REQUIRE(std::isinf(inf_vec.y()));
  REQUIRE(std::isinf(inf_vec.z()));
}
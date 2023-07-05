#include "edge.hpp"
#include "vec3.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("Edge parameters are correct", "[parameters]")
{
  Vec3 start(0, 0, 0);
  Vec3 end(1, 2, 3);
  Edge edge(start, end);
  float correct_norm = std::pow(1 + 4 + 9, 0.5);

  SECTION("Length") {
    REQUIRE(std::abs(edge.length() - correct_norm < 0.001));
  }
  SECTION("Vectors") {
    REQUIRE(edge.vec() == Vec3(1, 2, 3));
  }
}

TEST_CASE("Distance calculations work", "[distance]")
{
  SECTION("Parallel, overlapping") {
    Edge e1(Vec3(0, 0, 0), Vec3(1, 0, 0));
    Edge e2(Vec3(0, 2, 0), Vec3(2, 2, 0));

    REQUIRE(e1.DistTo(e1) == 0);
    REQUIRE(e1.DistTo(e2) == 2);
  }
  SECTION("Parallel, non-overlapping") {
    Edge e1(Vec3(0, 0, 0), Vec3(1, 0, 0));
    Edge e2(Vec3(-2, 0, 0), Vec3(-1, 0, 0));
    REQUIRE(e1.DistTo(e2) == 1);
  }
  SECTION("Non-parallel") {
    Edge e1(Vec3(0, 0, 2), Vec3(1, 2, 2));
    Edge e2(Vec3(0, 0, 0), Vec3(2, 0, 0));
    REQUIRE(e1.DistTo(e2) == 2);
  }
}
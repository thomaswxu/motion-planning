#include "obstacle.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("Collision detection works", "[collision]")
{
  Obstacle obs(-3, 3, -3, 3, -3, 3);

  SECTION("Point") {
    REQUIRE(obs.PointIsInside(Vec3(0, 0, 0)));
    REQUIRE(obs.PointIsInside(Vec3(3, 3, 3)));
    REQUIRE(obs.PointIsInside(Vec3(-3, 0, 0)));

    REQUIRE(!obs.PointIsInside(Vec3(5, 5, 5)));
  }
  SECTION("Edge") {
    float collision_check_step_mm = 1;
    REQUIRE(obs.EdgeIsInside(Edge(Vec3(0, 0, 0), Vec3(4, 4, 4)), collision_check_step_mm));
    REQUIRE(obs.EdgeIsInside(Edge(Vec3(-10, 0, 0), Vec3(-3, 0, 0)), collision_check_step_mm));

    REQUIRE(!obs.EdgeIsInside(Edge(Vec3(5, 0, 0), Vec3(10, 0, 0)), collision_check_step_mm));
  }
}
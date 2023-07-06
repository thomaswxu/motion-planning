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
}
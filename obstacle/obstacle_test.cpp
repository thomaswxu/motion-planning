#include "obstacle.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("Collision detection works", "[collision]")
{
  Obstacle obs(-3, 3, -3, 3, -3, 3);
  ArmDimensions arm_dims("/motion-planning/config/arm_dimensions.json");

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
  SECTION("Pose") {
    Obstacle big_obs(-200, 1000, 250, 500, -245, 300);
    Pose good_pose(-118.553, 13.512, 30.279, 77.908, -58.695, 14.492);
    Pose bad_pose(-92.713, -28.129, 62.892, 93.301, -99.490, -20.937);
    REQUIRE(!big_obs.PoseIsInside(good_pose, arm_dims));
    REQUIRE(big_obs.PoseIsInside(bad_pose, arm_dims));
  }
}
#include "obstacle.hpp"

#include <catch2/catch_test_macros.hpp>

#include <vector>

TEST_CASE("Construction works", "[construction]")
{
  SECTION("Explicit") {
    Obstacle obs(1, 2, 3, 4, 5, 6);
    REQUIRE(obs.min_y() == 3);
    REQUIRE(obs.max_z() == 6);
    REQUIRE(obs.max().Component(1) == 4);
  }
  SECTION("From configuration file") {
    Obstacle ceiling("/motion-planning/config/obstacles.json", "ceiling");
    Obstacle test_box("/motion-planning/config/obstacles.json", "test_box");
    REQUIRE(ceiling.name() == "ceiling");
    REQUIRE(test_box.name() == "test_box");

    REQUIRE(ceiling.min_y() == -1000);
    REQUIRE(ceiling.max_x() == 1500);

    REQUIRE(test_box.min_x() == 0);
    REQUIRE(test_box.max_z() == 1000);
  }
}

TEST_CASE("Reading multiple obstacles from configuration file works", "[parsing]")
{
  std::vector<Obstacle> obstacles = Obstacle::ObstaclesFromConfigFile("/motion-planning/config/obstacles.json");
  REQUIRE(obstacles.size() == 2);
  REQUIRE(obstacles[0].name() == "ceiling");
  REQUIRE(obstacles[1].name() == "test_box");

  REQUIRE(obstacles[0].max_y() == 1000);
  REQUIRE(obstacles[1].max_y() == -500);
}

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
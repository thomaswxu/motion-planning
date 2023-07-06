#include "pose_points.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("Computes arm XYZ points correctly", "[forward kinematics]")
{
  ArmDimensions arm_dims("/motion-planning/config/arm_dimensions.json");

  PosePoints pp_zero_pose(Pose(0, 0, 0, 0, 0, 0), arm_dims);
  REQUIRE(*(pp_zero_pose.arm_points().rbegin() + 1) == Vec3(700, -150, 710));

  PosePoints pp_random(Pose(50, -30, 50, 150, 100, -100), arm_dims);
  printf("pp random:\n");
  for (const Vec3& arm_pt : pp_random.arm_points()) {
    printf("[%.3f, %.3f, %.3f]\n", arm_pt.x(), arm_pt.y(), arm_pt.z());
  }
  REQUIRE(*(pp_random.arm_points().rbegin() + 1) == Vec3(-146.156, 150.479, 967.753));
}
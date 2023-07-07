#include "pose.hpp"

#include <catch2/catch_test_macros.hpp>

#include <vector>

TEST_CASE("Construction with vector works", "[construction]")
{
  std::vector<float> joint_values = {1, 2, 3, 4, 5, 6};
  Pose pose(joint_values);
  REQUIRE(pose.J3_deg == 3);
}

TEST_CASE("Operators work", "[operators]")
{
  Pose p1(1, 2, 3, 4, 5, 6);
  Pose p2(0, 0, 0, 0, 0, 0);
  Pose p3(1, 2.000001, 3, 3.9999999, 5, 6);

  REQUIRE(p1 == p1);
  REQUIRE(p1 != p2);
  REQUIRE(p1 == p3);
}

TEST_CASE("Joint distance works", "[distance]")
{
  Pose p1(1, 1, 1, 1, 1, 1);
  Pose p2(1, 2, 3, 4, 5, 6);
  Pose p3(0, 0, 0, 0, 0, 0);

  REQUIRE(p1.JointDistTo(p1) == 0);
  REQUIRE(p3.JointDistTo(p3) == 0);
  REQUIRE(p1.JointDistTo(p3) == 6);
  REQUIRE(p3.JointDistTo(p2) == 21);
}
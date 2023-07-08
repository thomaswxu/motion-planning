#include "arm_dimensions.hpp"

#include <string>

#include <catch2/catch_test_macros.hpp>

TEST_CASE("Config file is parsed correctly", "[parsing]")
{
  std::string config_file = "/motion-planning/config/arm_dimensions.json";
  ArmDimensions ad(config_file);

  SECTION("Link dimensions") {
    REQUIRE(ad.L1_mm == 245.0);
    REQUIRE(ad.L6_mm == 150.0);
    REQUIRE(ad.L_radius_mm == 85.0);
  }
  SECTION("End effector dimensions") {
    REQUIRE(ad.EE_length_mm == 100.0);
  }
  SECTION("Joint ranges") {
    REQUIRE(ad.J1_deg.first == -180);
    REQUIRE(ad.J1_deg.second == 180);
    REQUIRE(ad.J4_deg.first == -190);
    REQUIRE(ad.J4_deg.second == 190);

    REQUIRE(ad.joint_limits_deg.size() == 6);
    REQUIRE(ad.joint_limits_deg[3].first == -190);
    REQUIRE(ad.joint_limits_deg[3].second == 190);
  }
}
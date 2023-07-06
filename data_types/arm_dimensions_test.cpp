#include "arm_dimensions.hpp"

#include <string>

#include <catch2/catch_test_macros.hpp>

TEST_CASE("Link dimensions are parsed correctly", "[parsing]")
{
  std::string config_file = "/motion-planning/config/arm_dimensions.json";
  ArmDimensions ad(config_file);
  REQUIRE(ad.L1_mm == 245.0);
  REQUIRE(ad.L6_mm == 150.0);
  REQUIRE(ad.L_radius_mm == 85.0);

  REQUIRE(ad.EE_length_mm == 100.0);
}
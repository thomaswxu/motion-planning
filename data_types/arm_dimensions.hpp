/**
 * Class to read and store robot arm dimensions.
 */
#pragma once

#include <string>
#include <utility> // pair
#include <vector>

class ArmDimensions
{
public:
  ArmDimensions(const std::string& config_file);

  // "LX" = Link X (ascending from base link). Links are modelled as cylinder/pill shapes with a given radius.
  float L1_mm;
  float L2_mm;
  float L3_mm;
  float L4_mm;
  float L5_mm;
  float L6_mm;
  float L7_mm;
  float L_radius_mm;
  float EE_length_mm;

  // "JX" = Joint X (ascending from base).
  std::pair<float, float> J1_deg;
  std::pair<float, float> J2_deg;
  std::pair<float, float> J3_deg;
  std::pair<float, float> J4_deg;
  std::pair<float, float> J5_deg;
  std::pair<float, float> J6_deg;

  std::vector<std::pair<float, float>> joint_limits_deg; // Additional vector form for convenience.
};
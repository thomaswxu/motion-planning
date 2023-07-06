/**
 * Class to read and store robot arm dimensions.
 */
#pragma once

#include <string>

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
  float EE_radius_mm;
};
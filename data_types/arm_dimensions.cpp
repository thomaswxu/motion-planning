#include "arm_dimensions.hpp"

#include <fstream>

#include "nlohmann/json.hpp"

using json = nlohmann::json;

ArmDimensions::ArmDimensions(const std::string& config_file)
{
  std::ifstream config_fstream(config_file);
  json dimension_data = json::parse(config_fstream);

  L1_mm = dimension_data["L1_mm"];
  L2_mm = dimension_data["L2_mm"];
  L3_mm = dimension_data["L3_mm"];
  L4_mm = dimension_data["L4_mm"];
  L5_mm = dimension_data["L5_mm"];
  L6_mm = dimension_data["L6_mm"];
  L7_mm = dimension_data["L7_mm"];
  L_radius_mm = dimension_data["L_radius_mm"];
}

#include "arm_dimensions.hpp"

#include <fstream>

#include "nlohmann/json.hpp"

using json = nlohmann::json;

ArmDimensions::ArmDimensions(const std::string& config_file)
{
  std::ifstream config_fstream(config_file);
  json dimension_data = json::parse(config_fstream);

  L1_mm = dimension_data["links_mm"]["L1"];
  L2_mm = dimension_data["links_mm"]["L2"];
  L3_mm = dimension_data["links_mm"]["L3"];
  L4_mm = dimension_data["links_mm"]["L4"];
  L5_mm = dimension_data["links_mm"]["L5"];
  L6_mm = dimension_data["links_mm"]["L6"];
  L7_mm = dimension_data["links_mm"]["L7"];
  L_radius_mm = dimension_data["links_mm"]["L_radius"];
  EE_length_mm = dimension_data["EE"]["length_mm"];

  J1_deg = std::make_pair(dimension_data["joints_deg"]["J1"]["min"], dimension_data["joints_deg"]["J1"]["max"]);
  J2_deg = std::make_pair(dimension_data["joints_deg"]["J2"]["min"], dimension_data["joints_deg"]["J2"]["max"]);
  J3_deg = std::make_pair(dimension_data["joints_deg"]["J3"]["min"], dimension_data["joints_deg"]["J3"]["max"]);
  J4_deg = std::make_pair(dimension_data["joints_deg"]["J4"]["min"], dimension_data["joints_deg"]["J4"]["max"]);
  J5_deg = std::make_pair(dimension_data["joints_deg"]["J5"]["min"], dimension_data["joints_deg"]["J5"]["max"]);
  J6_deg = std::make_pair(dimension_data["joints_deg"]["J6"]["min"], dimension_data["joints_deg"]["J6"]["max"]);

  joint_limits_deg.assign({J1_deg, J2_deg, J3_deg, J4_deg, J5_deg, J6_deg});
}

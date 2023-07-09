/**
 * Simple example executable showing the minimal steps needed to generate a map and plan a robot arm path.
 */
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

#include "map.hpp"
#include "pose.hpp"

void PlanPath() {
  // Required parameters
  std::string arm_dimensions_config_file = "/motion-planning/config/arm_dimensions.json";
  std::string obstacles_config_file = "/motion-planning/config/obstacles.json";
  int number_of_nodes_for_map_generation = 1000;
  int number_of_node_neighbors_for_map_generation = 5;

  // Map construction
  Map map(number_of_nodes_for_map_generation,
          number_of_node_neighbors_for_map_generation,
          arm_dimensions_config_file,
          obstacles_config_file);

  // Planning a single path
  Pose arm_starting_pose(0, 0, 0, 0, 0, 0);
  Pose arm_goal_pose(-120, 30, 0, 100, -100, 30);
  std::vector<Pose> planned_path = map.PlanPath(arm_starting_pose, arm_goal_pose);

  // Saving the path for visualization
  std::string path_save_file = "/motion-planning/saved_path.txt";
  map.SavePath(planned_path, path_save_file);
}

int main(int argc, char** argv) {
  try {
    PlanPath();
  }
  catch (const std::runtime_error& runtime_error) {
    printf("Failed to plan a path because: %s\n", runtime_error.what());
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
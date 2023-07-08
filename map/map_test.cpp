#include "map.hpp"

#include "arm_dimensions.hpp"
#include "obstacle.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("Map construction works", "[construction]")
{
  ArmDimensions arm_dimensions("/motion-planning/config/arm_dimensions.json");
  std::vector<Obstacle> obstacles = {};
  int num_node_neighbors = 1;

  SECTION("Correct number of nodes") {
    int num_nodes = 10;
    Map map(num_nodes, num_node_neighbors, arm_dimensions, obstacles);
    REQUIRE(map.Size() == num_nodes);
  }
  SECTION("Correct number of nodes when given negative number") {
    int num_nodes = -10;
    Map map(num_nodes, num_node_neighbors, arm_dimensions, obstacles);
    REQUIRE(map.Size() == 0);
  }
  SECTION("Nodes are in order") {
    int num_nodes = 100;
    Map map(num_nodes, num_node_neighbors, arm_dimensions, obstacles);
    bool node_ids_match = true;
    for (int i = 0; i < num_nodes; i++) {
      if(map.nodes()[i]->id() != i) {
        node_ids_match = false;
        break;
      }
    }
    REQUIRE(node_ids_match);
  }
}

TEST_CASE("Path planning works", "[planning]")
{
  ArmDimensions arm_dimensions("/motion-planning/config/arm_dimensions.json");
  int num_nodes = 500;
  int num_node_neighbors = 3;

  SECTION("No obstacles") {
    Map map(num_nodes, num_node_neighbors, arm_dimensions, {});
    Pose start(0, 0, 0, 0, 0, 0);
    Pose goal(15, 24, 100, -35, 24, 160);
    std::vector<Pose> planned_path = map.PlanPath(start, goal);
    REQUIRE(!planned_path.empty());
  }
  SECTION("Obstacles") {

  }
}
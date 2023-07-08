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
    for (int i = 0; i < num_nodes; i++) {
      REQUIRE(map.nodes()[i]->id() == i);
    }
  }
}
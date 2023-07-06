#include "node.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("Connection functions work", "[connection]")
{
  Node node(Pose(0, 0, 0, 0, 0, 0), 0);
  REQUIRE(node.NumConnections() == 0);

  // Addition
  node.AddConnection(1);
  node.AddConnection(2);
  node.AddConnection(3);
  node.AddConnection(4);
  node.AddConnection(5);
  REQUIRE(node.NumConnections() == 5);

  // Connection order
  std::vector<int> correct_connections = {1, 2, 3, 4, 5};
  REQUIRE(node.connection_ids() == correct_connections);

  // Connection status
  REQUIRE(!node.ConnectedTo(6));
  REQUIRE(node.ConnectedTo(3));

  // Removal
  node.RemoveConnection(1);
  node.RemoveConnection(4);
  REQUIRE(node.NumConnections() == 3);

  std::vector<int> correct_connections_after_removal = {2, 3, 5};    
  REQUIRE(node.connection_ids() == correct_connections_after_removal);
}
/**
 * 3D map class, implementing a generation method (e.g. Probabilistic roadmap) and a planning method (e.g. A*) to 
 * do planning queries.
 */
#pragma once

#include "arm_dimensions.hpp"
#include "node.hpp"
#include "obstacle.hpp"

#include <memory>
#include <random>
#include <queue>
#include <vector>

class Map
{
public:

private:
  int num_nodes_;
  int num_node_neighbors_; // For use during roadmap generation.
  
  const ArmDimensions arm_dimensions_;

  std::vector<std::shared_ptr<Node>> nodes_; // All valid nodes, ordered by node ID, ascending.
  std::vector<Obstacle> obstacles_;

  // For random generation
  std::random_device random_device_;
  std::mt19937 random_engine_; // Mersenne-Twister
  std::vector<std::uniform_real_distribution<float>> joint_uniform_distributions_;
};
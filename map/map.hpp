/**
 * 3D map class, implementing a generation method (e.g. Probabilistic roadmap) and a planning method (e.g. A*) to 
 * do planning queries.
 *
 * Currently uses uniform sampling for random, A*, and LazyPRM.
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

  /** Construct a roadmap using the provided parameters. May take awhile, depending on the params.*/
  Map(int num_nodes, int num_node_neighbors, const ArmDimensions& arm_dimensions,
      const std::vector<Obstacle>& obstacles);

  /** Get a new random arm pose.*/
  Pose RandomPose();
  
  /** Check whether a given pose is collision-free (including self collisions).*/
  bool PoseIsFree(const Pose& pose) const;

  /** Check whether a pose collides with itself, based on stored arm dimensions.*/
  bool PoseSelfCollides(const Pose& pose) const;

  /** Get the "nearest" nodes for a given node, sorted ascending by distance. Number determined by member variable.*/
  std::vector<std::shared_ptr<Node>> NearNodes(const Node& node) const;

  inline int Size() const { return nodes_.size(); }
  inline std::vector<std::shared_ptr<Node>> nodes() const { return nodes_; }

private:
  /** Initialize random distributions used for map generation.*/
  void InitializeJointSamplingDistributions();

  /** Generate some nodes and add to map. Could be made public to allow additional calls.*/
  void GenerateNodes(int num_nodes);

  /** Generate edges to connect all stored nodes.*/
  void GenerateEdges();

  int num_nodes_;
  int num_node_neighbors_; // For use during roadmap generation.
  
  const ArmDimensions arm_dimensions_;

  std::vector<std::shared_ptr<Node>> nodes_; // All valid nodes, ordered by node ID, ascending.
  std::vector<Obstacle> obstacles_;

  // For random generation
  std::random_device random_device_;
  std::mt19937 random_engine_; // Mersenne-Twister
  std::vector<std::uniform_real_distribution<float>> joint_distributions_;
};
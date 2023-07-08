/**
 * 3D map class, implementing a generation method (e.g. Probabilistic roadmap) and a planning method (e.g. A*) to 
 * do planning queries.
 *
 * Currently uses uniform sampling for random, A*, and LazyPRM.
 */
#pragma once

#include "arm_dimensions.hpp"
#include "node.hpp"
#include "node_comparator.hpp"
#include "obstacle.hpp"

#include <memory>
#include <queue>
#include <random>
#include <string>
#include <vector>

class Map
{
public:

  /** Construct a roadmap using the provided parameters. May take awhile, depending on the params.*/
  Map(int num_nodes, int num_node_neighbors, const ArmDimensions& arm_dimensions,
      const std::vector<Obstacle>& obstacles);
  /** Constructor that reads dimensions and obstacles from provided configuration files.*/
  Map(int num_nodes, int num_node_neighbors, const std::string& arm_dimensions_config_file,
      const std::string& obstacles_config_file);

  /** Plan a collision-free path from given start and goal poses.*/
  std::vector<Pose> PlanPath(const Pose& start, const Pose& goal) const;

  /** Get a new random arm pose.*/
  Pose RandomPose();
  
  /** Check whether a pose collides with itself, based on stored arm dimensions.*/
  bool PoseSelfCollides(const Pose& pose) const;
  bool PoseSelfCollides(const PosePoints& pose_points) const;

  /** Check whether a given pose is collision-free (including self collisions).*/
  bool PoseIsFree(const Pose& pose) const;

  /** Check whether the "edge" connecting two poses is collision-free.*/
  bool PoseEdgeIsFree(const Pose& pose1, const Pose& pose2) const;

  /** Compute intermediate poses evenly divided between two given poses, in order.*/
  static std::vector<Pose> GetIntermediatePoses(const Pose& pose1, const Pose& pose2,
                                                float joint_angle_step_deg=kMaxPoseStep_deg);

  /** Get the "nearest" nodes for a given node, sorted ascending by distance. Number determined by member variable.*/
  std::vector<std::shared_ptr<Node>> NearNodes(const Node& node) const;

  inline int Size() const { return nodes_.size(); }
  inline std::vector<std::shared_ptr<Node>> nodes() const { return nodes_; }
  inline std::vector<Obstacle> obstacles() const { return obstacles_; }

  static const int kMaxPoseStep_deg = 10; // Max step to take when constructing pose paths.

private:
  /** Initialize random distributions used for map generation.*/
  void InitializeJointSamplingDistributions();

  /** Generate some nodes and add to map. Could be made public to allow additional calls.*/
  void GenerateNodes(int num_nodes);

  /** Generate edges to connect all stored nodes.*/
  void GenerateEdges();

  /** Check whether a node path is valid. If invalid, prunes invalid edge and resets nodes for a new planning attempt.*/
  bool CheckNodePath(const std::vector<Node>& node_path) const;

  /** Do a single planning attempt. Returns full path (including start/goal nodes) if successful, empty otherwise.*/
  std::vector<Node> PlanPathAttempt(const Node& start_node, const Node& goal_node) const;

  /** Check whether a given node has already been added to the priority queue.*/
  bool NodeIsInQueue(const Node& node, std::priority_queue<Node, std::vector<Node>, NodeComparator> node_queue) const;

  /** Retrieve the full node path between the two given nodes (inclusive).*/
  std::vector<Node> GetNodePath(const Node& last_node, const Node& first_node) const;

  /** Convert a sequence of nodes to the corresponding sequence of arm poses.*/
  std::vector<Pose> NodePathToPosePath(const std::vector<Node>& node_path) const;

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
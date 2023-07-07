#include "map.hpp"

#include <algorithm>
#include <chrono>

Map::Map(int num_nodes, int num_node_neighbors, const ArmDimensions& arm_dimensions, 
         const std::vector<Obstacle>& obstacles)
    : num_nodes_(num_nodes), num_node_neighbors_(num_node_neighbors),
      arm_dimensions_(arm_dimensions), obstacles_(obstacles)
{
  InitializeJointSamplingDistributions();
  GenerateNodes(num_nodes);
  GenerateEdges();
}

Pose Map::RandomPose()
{
  // Note: non const function because std::uniform_real_distribution::operator() not const.
  std::vector<float> joint_values;
  for (std::uniform_real_distribution<float>& joint_distribution : joint_distributions_) {
    float joint_val = joint_distribution(random_engine_);
    joint_values.push_back(joint_val);
  }
  return Pose(joint_values);
}

bool Map::PoseIsFree(const Pose& pose) const
{
  PosePoints pose_points(pose, arm_dimensions_);
  if (PoseSelfCollides(pose_points)) {
    return false;
  }

  std::vector<Vec3> inverse_vectors = pose_points.ArmEdgeInverseVectors();
  for (const Obstacle& obstacle : obstacles_) {
    if (obstacle.PoseIsInside(pose_points, inverse_vectors, arm_dimensions_)) {
      return false;
    }
  }
  return true;
}

bool Map::PoseSelfCollides(const Pose& pose) const
{
  return PoseSelfCollides(PosePoints(pose, arm_dimensions_));
}
bool Map::PoseSelfCollides(const PosePoints& pose_points) const
{
  // Basic self-collision checking.
  // Currently assumes that self-collisions are only possible involving last 2 links or end effector.
  std::vector<Edge> arm_edges = pose_points.arm_edges();
  Edge L1 = arm_edges.front();
  Edge L2 = arm_edges[1];
  Edge L3 = arm_edges[2];
  Edge L6 = arm_edges[5];
  Edge L7 = arm_edges[6];
  Edge EE_link = arm_edges.back();

  // Minimum distance to prevent collision between links, based on "pill" shape links.
  float minimum_distance_mm = 2 * arm_dimensions_.L_radius_mm;
  
  if (L6.DistTo(L2) < minimum_distance_mm || L6.DistTo(L3) < minimum_distance_mm) {
    return true;
  }
  if (L7.DistTo(L2) < minimum_distance_mm || L7.DistTo(L3) < minimum_distance_mm) {
    return true;
  }
  if (EE_link.DistTo(L1) < minimum_distance_mm
   || EE_link.DistTo(L2) < minimum_distance_mm
   || EE_link.DistTo(L3) < minimum_distance_mm) {
    return true;
  }
  return false;
}

std::vector<std::shared_ptr<Node>> Map::NearNodes(const Node& node) const
{
  // Sort all nodes based on distance and return the closest few (excluding the same node itself).
  std::vector<std::shared_ptr<Node>> sorted_near_nodes = nodes_;
  std::sort(sorted_near_nodes.begin(), sorted_near_nodes.end(),
            [&node](const std::shared_ptr<Node>& node1, const std::shared_ptr<Node>& node2) {
              return node.DistTo(*node1) < node.DistTo(*node2);
            });
  std::vector<std::shared_ptr<Node>>::const_iterator first_near_node = sorted_near_nodes.begin() + 1;
  std::vector<std::shared_ptr<Node>>::const_iterator last_near_node =
    first_near_node + std::min(num_node_neighbors_, int(sorted_near_nodes.size() - 1));
  return std::vector<std::shared_ptr<Node>>(first_near_node, last_near_node);
}

void Map::InitializeJointSamplingDistributions()
{
  for (const std::pair<float, float>& joint_limit : arm_dimensions_.joint_limits_deg) {
    joint_distributions_.push_back(std::uniform_real_distribution<float>(joint_limit.first, joint_limit.second));
  }
}

void Map::GenerateNodes(int num_nodes)
{
  printf("Map: Generating nodes...");
  auto start_time = std::chrono::steady_clock::now();

  int starting_node_id = nodes_.size();
  for (int node_id = starting_node_id; node_id < starting_node_id + num_nodes;) {
    Pose random_pose = RandomPose();
    if (PoseIsFree(random_pose)) {
      nodes_.push_back(std::make_shared<Node>(Node(random_pose, node_id)));
      node_id++;
    }
  }
  float generation_time_s =
    std::chrono::duration_cast<std::chrono::seconds>((std::chrono::steady_clock::now() - start_time)).count();
  printf("Map: Generated %d nodes in %.2f seconds.\n", num_nodes, generation_time_s);
}

void Map::GenerateEdges()
{
  printf("Map: Generating edges...");
  auto start_time = std::chrono::steady_clock::now();

  for (const std::shared_ptr<Node>& node : nodes_) {
    for (const std::shared_ptr<Node>& near_node : NearNodes(*node)) {
      node->AddConnection(near_node->id()); // Don't bother with collision check here because LazyPRM.
    }
  }

  float generation_time_s =
    std::chrono::duration_cast<std::chrono::seconds>((std::chrono::steady_clock::now() - start_time)).count();
  printf("Map: Generated node edges in %.2f seconds.\n", generation_time_s);
}

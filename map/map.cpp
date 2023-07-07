#include "map.hpp"

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
  return true;
}

bool Map::PoseSelfCollides(const Pose& pose) const
{
  return false;
}

// std::vector<std::shared_ptr<Node>> Map::NearNodes(const Node& node) const
// {

// }

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

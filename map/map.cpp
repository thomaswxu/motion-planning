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

std::vector<Pose> Map::PlanPath(const Pose& start, const Pose& goal) const
{
  printf("Map: Planning path...\n");
  auto start_time = std::chrono::steady_clock::now();

  // Check start/goal poses
  if (!start.IsValid(arm_dimensions_) || !goal.IsValid(arm_dimensions_)) {
    printf("Map: Cannot plan path because at least one of given start/goal poses are invalid.\n");
    return {};
  }
  if (!PoseIsFree(start) || !PoseIsFree(goal)) {
    printf("Map: Cannot plan path because at least one of given start/goal poses are not collision-free.\n");
    return {};
  }

  // Add start pose to graph
  Node start_node = Node(start, Node::kStartNodeID);
  start_node.set_distance_from_start(0);
  for (const std::shared_ptr<Node>& near_node : NearNodes(start_node)) {
    if (PoseEdgeIsFree(near_node->pose(), start_node.pose())) {
      start_node.AddConnection(near_node->id());
    }
  }
  // Add goal pose to graph
  Node goal_node = Node(goal, Node::kGoalNodeID);
  int num_goal_node_connections;
  for (const std::shared_ptr<Node>& near_node : NearNodes(goal_node)) {
    if (PoseEdgeIsFree(near_node->pose(), goal_node.pose())) {
      near_node->AddConnection(goal_node.id());
      // Note: ideally should re-sort near_nodes list of neighbors here; most likely negligible.
      num_goal_node_connections++;
    }
  }
  printf("Map: Start node neighbors: %d. Goal node neighbors: %d.\n",
          start_node.NumConnections(), num_goal_node_connections);
  if (start_node.NumConnections() == 0 || num_goal_node_connections == 0) {
    printf("Map: Cannot plan path because at least one of given start/goal nodes have no neighbors.\n");
    return {};    
  }

  // Main planning loop (Lazy_PRM: keep retrying and pruning edges with collisions until successful)
  std::vector<Node> node_path;
  bool node_path_is_free;
  while (!node_path_is_free) {
    node_path_is_free = true;
    node_path = PlanPathAttempt(start_node, goal_node);
    if (node_path.empty()) {
      break;
    }
    if (node_path.size() == 2) {
      // Direct from start to goal node (already checked for collisions)
      break;
    }
    node_path_is_free = CheckNodePath(node_path);
  }

  if (node_path.empty()) {
    printf("Map: Failed to plan path.\n");
    return {};
  }

  float planning_time_s =
    std::chrono::duration_cast<std::chrono::seconds>((std::chrono::steady_clock::now() - start_time)).count();
  printf("Map: Generated path in %.2f s.\n", planning_time_s);
  return NodePathToPosePath(node_path);
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

bool Map::PoseEdgeIsFree(const Pose& pose1, const Pose& pose2) const
{
  // Precompute intermediate poses (along with related quantities)
  std::vector<Pose> intermediate_poses = GetIntermediatePoses(pose1, pose2);
  std::vector<PosePoints> all_pose_points;
  std::vector<std::vector<Vec3>> all_inverse_vectors;
  for (const Pose& intermediate_pose : intermediate_poses) {
    PosePoints pose_points(intermediate_pose, arm_dimensions_);
    if (PoseSelfCollides(pose_points)) {
      return false;
    }
    all_pose_points.push_back(pose_points);
    all_inverse_vectors.push_back(pose_points.ArmEdgeInverseVectors());
  }

  // Check obstacle collisions
  for (const Obstacle& obstacle : obstacles_) {
    if (obstacle.PoseEdgeIsInside(intermediate_poses, all_pose_points, all_inverse_vectors, arm_dimensions_)) {
      return false;
    }
  }
  return true;
}

std::vector<Pose> Map::GetIntermediatePoses(const Pose& pose1, const Pose& pose2, float joint_angle_step_deg)
{
  std::vector<Pose> intermediate_poses;
  int num_steps = std::ceil(pose1.MaxJointDistTo(pose2) / joint_angle_step_deg);
  float inverse_num_steps = 1.0 / num_steps;
  for (int i = 0; i <= num_steps; i++) {
    intermediate_poses.push_back(
      Pose(pose1.J1_deg + i * (pose2.J1_deg - pose1.J1_deg) * inverse_num_steps,
           pose1.J2_deg + i * (pose2.J2_deg - pose1.J2_deg) * inverse_num_steps,
           pose1.J3_deg + i * (pose2.J3_deg - pose1.J3_deg) * inverse_num_steps,
           pose1.J4_deg + i * (pose2.J4_deg - pose1.J4_deg) * inverse_num_steps,
           pose1.J5_deg + i * (pose2.J5_deg - pose1.J5_deg) * inverse_num_steps,
           pose1.J6_deg + i * (pose2.J6_deg - pose1.J6_deg) * inverse_num_steps));
  }
  return intermediate_poses;
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
  printf("Map: Generating nodes...\n");
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
  printf("Map: Generating edges...\n");
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

bool Map::CheckNodePath(const std::vector<Node>& node_path) const
{
  Node previous_node;
  bool set_previous_node;
  for (const Node& node : node_path) {
    // Skip start node
    if (!set_previous_node) {
      previous_node = node;
      set_previous_node = true;
      continue;
    }

    // Check for edges/connections with collisions
    // (No need to check start/goal node edges for collisions again)
    if (previous_node.id() != Node::kStartNodeID
        && node.id() != Node::kGoalNodeID
        && !PoseEdgeIsFree(previous_node.pose(), node.pose())) {
      // Prune bad connection
      nodes_[previous_node.id()]->RemoveConnection(node.id());
      nodes_[node.id()]->RemoveConnection(previous_node.id());
      
      // Reset nodes for a new planning attempt
      for (const std::shared_ptr<Node>& node : nodes_) {
        node->set_distance_from_start(Node::kInitialDistFromStart);
        node->set_parent_id(Node::kInitialNodeID);
      }
      return false;
    }
    previous_node = node;
  }
  return true;
}

std::vector<Node> Map::PlanPathAttempt(const Node& start_node, const Node& goal_node) const
{
  // Initialize node priority queue
  NodeComparator node_comparator(goal_node);
  std::priority_queue<Node, std::vector<Node>, NodeComparator> node_queue(node_comparator);
  node_queue.push(start_node);

  // Plan path via A*
  std::vector<Node> node_path; // Will include start/goal node
  while (!node_queue.empty()) {
    Node current_node = node_queue.top();
    if (current_node.id() == Node::kGoalNodeID) {
      // Successfully reached goal; step back through parents to make whole path.
      node_path = GetNodePath(current_node, start_node);
      break;
    }
    node_queue.pop();

    for (const int connected_node_id : current_node.connection_ids()) {
      std::shared_ptr<Node> connected_node = 
        connected_node_id != Node::kGoalNodeID ? nodes_[connected_node_id] : std::make_shared<Node>(goal_node);
      
      float tentative_distance_from_start = current_node.distance_from_start() + current_node.DistTo(*connected_node);
      if (tentative_distance_from_start < connected_node->distance_from_start()) {
        connected_node->set_parent_id(current_node.id());
        connected_node->set_distance_from_start(tentative_distance_from_start);
        if (!NodeIsInQueue(*connected_node, node_queue)) {
          node_queue.push(*connected_node);
        }
      }
    }
  }
  return node_path;
}

bool Map::NodeIsInQueue(const Node& node, std::priority_queue<Node, std::vector<Node>, NodeComparator> node_queue) const
{
  while (!node_queue.empty()) {
    if (node_queue.top().id() == node.id()) {
      return true;
    }
    node_queue.pop();
  }
  return false;
}

std::vector<Node> Map::GetNodePath(const Node& last_node, const Node& first_node) const
{
  std::vector<Node> node_path = {last_node};
  Node node = last_node;
  while (node.parent_id() != first_node.id()) {
    node = *nodes_[node.parent_id()];
    node_path.push_back(node);
  }
  node_path.push_back(first_node);
  return std::vector<Node>(node_path.rbegin(), node_path.rend());
}

std::vector<Pose> Map::NodePathToPosePath(const std::vector<Node>& node_path) const
{
  std::vector<Pose> pose_path;
  Node previous_node;
  bool set_previous_node;
  for (const Node& node : node_path) {
    if (!set_previous_node) {
      previous_node = node;
      set_previous_node = true;
      continue;
    }
    float max_joint_difference_deg = node.pose().MaxJointDistTo(previous_node.pose());
    int num_steps = std::ceil(max_joint_difference_deg / kMaxPoseStep_deg);
    float inverse_num_steps = 1.0 / num_steps;
    for (int i = 0; i < num_steps; i++) {
      pose_path.push_back(
        Pose(previous_node.pose().J1_deg + i * (node.pose().J1_deg - previous_node.pose().J1_deg) * inverse_num_steps,
             previous_node.pose().J2_deg + i * (node.pose().J2_deg - previous_node.pose().J2_deg) * inverse_num_steps,
             previous_node.pose().J3_deg + i * (node.pose().J3_deg - previous_node.pose().J3_deg) * inverse_num_steps,
             previous_node.pose().J4_deg + i * (node.pose().J4_deg - previous_node.pose().J4_deg) * inverse_num_steps,
             previous_node.pose().J5_deg + i * (node.pose().J5_deg - previous_node.pose().J5_deg) * inverse_num_steps,
             previous_node.pose().J6_deg + i * (node.pose().J6_deg - previous_node.pose().J6_deg) * inverse_num_steps));
    }
    previous_node = node;
  }
  pose_path.push_back(previous_node.pose()); // Add very last node's pose
  return pose_path;
}
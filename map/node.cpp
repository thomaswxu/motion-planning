#include "node.hpp"

#include <algorithm> // find, remove
#include <limits>

const float Node::kInitialDistFromStart = std::numeric_limits<float>::infinity();

Node::Node()
    : pose_(Pose(0, 0, 0, 0, 0, 0)), distance_from_start_(kInitialDistFromStart)
{}

Node::Node(const Pose& pose, int id)
    : pose_(pose), id_(id), distance_from_start_(kInitialDistFromStart)
{}

void Node::AddConnection(int node_id)
{
  connection_ids_.push_back(node_id);
}

void Node::RemoveConnection(int node_id)
{
  std::vector<int>::iterator new_end = std::remove(connection_ids_.begin(), connection_ids_.end(), node_id);
  connection_ids_.erase(new_end, connection_ids_.end());
}

bool Node::ConnectedTo(int node_id) const
{
  return std::find(connection_ids_.begin(), connection_ids_.end(), node_id) != connection_ids_.end();
}

float Node::DistTo(const Node& node) const
{
  return pose_.JointDistTo(node.pose());
}

void Node::Reset()
{
  pose_.Clear();
  id_ = -1;
  connection_ids_.clear();

  distance_from_start_ = kInitialDistFromStart;
  parent_id_ = -1;
}
#include "node_comparator.hpp"

NodeComparator::NodeComparator(const Node& goal_node)
    : goal_node_(goal_node)
{}

bool NodeComparator::operator() (const Node& node1, const Node& node2) const
{
  return node1.distance_from_start() + node1.DistTo(goal_node_)
       > node2.distance_from_start() + node2.DistTo(goal_node_);
}
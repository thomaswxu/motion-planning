/**
 * Custom comparator class for sorting nodes during path planning.
 */
#pragma once

#include "node.hpp"

class NodeComparator
{
public:
  NodeComparator(const Node& goal_node);
  bool operator() (const Node& node1, const Node& node2) const;

private:
  Node goal_node_;
};
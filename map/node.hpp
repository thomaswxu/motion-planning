/**
 * Simple node class used to form generated graphs/maps.
 */
#pragma once

#include "pose.hpp"

class Node
{
public:
  Node();
  Node(const Pose& pose, int id);

  /** Add a connected node to this one.*/
  void AddConnection(int node_id);

  /** Remove a connected node from list of connections.*/
  void RemoveConnection(int node_id);

  /** Check whether a given node is connected to this one.*/
  bool ConnectedTo(int node_id) const;

  /** Return the path planning "distance" to a given node.*/
  float DistTo(const Node& node) const;

  /** Clear all parameters to default starting values.*/
  void Reset();

  /** Get the current number of connected nodes.*/
  inline int NumConnections() const { return connection_ids_.size(); }

  inline Pose pose() const { return pose_; }
  inline int id() const { return id_; }
  inline std::vector<int> connection_ids() const { return connection_ids_; }
  inline float distance_from_start() const { return distance_from_start_; }
  inline int parent_id() const { return parent_id_; }

  inline void set_distance_from_start(float distance) { distance_from_start_ = distance; }
  inline void set_parent_id(int id) { parent_id_ = id; }

  static const float kInitialDistFromStart;
  static const int kInitialNodeID = -1;
  static const int kStartNodeID = -2;
  static const int kGoalNodeID = -3;

private:
  Pose pose_;
  int id_ = -1;
  std::vector<int> connection_ids_; // Connected nodes, should be ordered by distance, ascending.

  float distance_from_start_;
  int parent_id_ = -1;
};
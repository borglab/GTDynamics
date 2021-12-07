/**
 * @file  RoadMap.h
 * @brief  Roadmap
 * @author Frank Dellaert
 * @author Antoni Jubes
 */

#pragma once

#include <gtsam/geometry/Pose3.h>

#include <utility>
#include <vector>

namespace gtdynamics {

class RoadMap {
 public:
  using Edge = std::pair<double, size_t>;

 private:
  std::vector<gtsam::Pose3> poses_;
  std::vector<gtsam::Vector7> states_;
  std::vector<size_t> statetopose_;                // many to one
  std::vector<std::vector<size_t>> posetostates_;  // one to many
  size_t lastupdatedpose_;                         // for
  double threshold_;  // edge weight threshold (max joint distance)
  std::vector<std::vector<Edge>> adjacencylist_;
  size_t num_maxpaths_;

 public:
  RoadMap();

  // get member methods
  const std::vector<gtsam::Pose3> getposes() { return poses_; }
  const std::vector<gtsam::Vector7> getstatenodes() { return states_; }
  const std::vector<std::vector<Edge>> getadjacencylist();

  // not direct get member methods
  size_t getPoseFromState(const size_t stateindex);
  const std::vector<size_t> getStatesFromPose(const size_t poseindex);

  // set member methods
  void set_threshold(double threshold) { threshold_ = threshold; }
  void set_num_maxpaths(size_t num_maxpaths) { num_maxpaths_ = num_maxpaths; }

  /**
   * @brief Adds poses to internal pose list
   *
   * @param poses -- poses to be added
   */
  void addPoseNodes(const std::vector<gtsam::Pose3>& poses);

  /**
   * @brief Uses IKFast wrapper to compute solutions from internal pose list and
   * adds them to internal state list. They will be the nodes in the roadmap
   * graph.
   * Theta7 is discretized first so to reduce the 7-DoF arm problem to a 6-DoF
   * one which IKFast then solves. Currently, it is sampled uniformly.
   *
   * @param theta7_samples -- Number of theta7 samples.
   */
  void computeStateSolutions(const size_t theta7_samples);

  /**
   * @brief Create a Graph from state nodes found previously. It puts an edge
   * between nodes if the distance in jointspace is smaller than a threshold
   * (defined in set_threshold)
   *
   */
  void createGraph();

  /**
   * @brief Get index of closest state node to a given state. The used distance
   * is in joint space
   *
   * @param state -- gtsam::Vector7 given state
   * @return size_t -- index of closest state node
   */
  size_t findClosestNodeState(const gtsam::Vector7& state);

  /**
   * @brief Given a pose, get the closest pose that is stored and then return
   * the indices of state nodes corresponding to this pose
   *
   * @param pose -- gtsam::Pose3 given pose
   * @return const std::vector<size_t>
   */
  const std::vector<size_t> findClosestNodesPose(const gtsam::Pose3& pose);

  /**
   * @brief Returns a vector of shortest paths (each being a vector of
   * waypoints, i.e., vector of node indices) from a starting source state node
   * to a set of nodes defined by one end_pose. 
   *
   * @param start_node
   * @param end_pose
   * @return std::vector<std::vector<size_t>>
   */
  std::vector<std::vector<size_t>> findWaypoints(const size_t start_node,
                                                 const size_t end_pose);
};
}  // namespace gtdynamics

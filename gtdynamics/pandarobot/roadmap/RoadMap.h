/**
 * @file  RoadMap.h
 * @brief  Roadmap
 * @author Frank Dellaert
 * @author Antoni Jubes
 */

#pragma once

#include <gtsam/geometry/Pose3.h>

#include <string>
#include <utility>
#include <vector>

namespace gtdynamics {

struct Heuristic;

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
  // vars used when computing solutions
  size_t theta7_samples_;
  size_t num_deleted_states_;
  size_t num_not_found_;
  std::array<size_t, 9> beforecheck_distribution;
  std::array<size_t, 9> aftercheck_distribution;

 public:
  RoadMap();

  // get member methods
  size_t getnumdeleted() const { return num_deleted_states_; }
  size_t getnumnotfound() const { return num_not_found_; }
  const std::array<size_t, 9> getbeforecheckdistribution() const {return beforecheck_distribution; }
  const std::array<size_t, 9> getaftercheckdistribution() const {return aftercheck_distribution; }
  const std::vector<gtsam::Pose3>& getposes() const { return poses_; }
  const std::vector<gtsam::Vector7>& getstatenodes() const { return states_; }
  const std::vector<std::vector<Edge>>& getadjacencylist() const;

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
  RoadMap& addPoseNodes(const std::vector<gtsam::Pose3>& poses);

  static bool checkJointLimits(const gtsam::Vector7& joint_state);

  RoadMap& addStateNodes(const std::vector<gtsam::Vector7>& states);

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
   * It does so naively, a O(n^2) complexity with n being the number of state
   * nodes
   *
   */
  void createGraph();  // createGraphNaively

  // createGraph(pose_locality);

  std::vector<std::vector<std::pair<size_t, size_t>>>
  computeStateLocality(
      const std::vector<std::vector<size_t>>& pose_locality,
      size_t theta_kernel_size = 1);

  void createGraphFromReference(
      const std::vector<std::vector<std::pair<size_t, size_t>>>& reference);

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
   * @brief
   *
   * @param waypoints
   * @return std::vector<std::vector<size_t>>
   */
  std::vector<std::vector<size_t>> findPath(
      const std::vector<std::vector<size_t>>& waypoint_sets, Heuristic* h,
      const std::string& save_path = "");

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

struct Node {
  size_t level, idx;
  Node(size_t l, size_t i) : level{l}, idx{i} {}
};
struct Heuristic {
  virtual void preprocess(const RoadMap& roadmap,
                          const std::vector<std::vector<size_t>>& waypoints);
  virtual double operator()(const RoadMap& roadmap, const Node& node);
};

struct DirectDistance : Heuristic {
  std::vector<size_t> end_nodes;
  void preprocess(const RoadMap& roadmap,
                  const std::vector<std::vector<size_t>>& waypoints) override;
  double operator()(const RoadMap& roadmap, const Node& node) override;
};

}  // namespace gtdynamics

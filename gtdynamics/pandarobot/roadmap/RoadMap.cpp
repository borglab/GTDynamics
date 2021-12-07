/**
 * @file  RoadMap.h
 * @brief  Roadmap
 * @author Frank Dellaert
 * @author Antoni Jubes
 */

#include "RoadMap.h"

#include <gtdynamics/pandarobot/ikfast/PandaIKFast.h>
#include <gtsam/geometry/Pose3.h>

#include <limits>
#include <queue>
#include <vector>

namespace gtdynamics {
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Vector7;

RoadMap::RoadMap() {
  poses_ = std::vector<Pose3>();
  states_ = std::vector<Vector7>();
  statetopose_ = std::vector<size_t>();
  posetostates_ = std::vector<std::vector<size_t>>();
  lastupdatedpose_ = 0;
  num_maxpaths_ = 1;
}

void RoadMap::addPoseNodes(const std::vector<Pose3>& poses) {
  // concatenate previous poses with given ones
  std::vector<Pose3> concat;
  concat.reserve(poses_.size() + poses.size());  // preallocate memory
  concat.insert(concat.end(), poses_.begin(), poses_.end());
  concat.insert(concat.end(), poses.begin(), poses.end());
  poses_ = concat;

  // add new pose indices to posetostates_
  std::vector<std::vector<size_t>> newposetostates(poses_.size() +
                                                   poses.size());
  newposetostates.insert(newposetostates.end(), posetostates_.begin(),
                         posetostates_.end());
  posetostates_ = newposetostates;
}

/**
 * For each pose (with corresponding pose_index), compute ik solutions with
 * all possible theta values and store:
 *    1) jointindex -> solution : in states_
 *    2) joint_index -> pose_index : in statetopose_
 *    3) pose_index -> [joint_index] : in posetostates_
 */
void RoadMap::computeStateSolutions(const size_t theta7_samples) {
  // Get the possible different values for theta7
  // TODO: factorize this with unittests, choose random or uniform sampling
  double theta7_l_lim = -2.8973, theta7_r_lim = 2.8973;
  double interval = theta7_r_lim - theta7_l_lim;
  std::vector<double> theta7_values(theta7_samples);
  for (size_t i = 1; i <= theta7_samples; ++i)
    theta7_values[i] = theta7_l_lim + i * interval / (theta7_samples + 1);

  for (size_t idx_pose = lastupdatedpose_; idx_pose < poses_.size();
       ++idx_pose) {
    posetostates_[idx_pose] = std::vector<size_t>();

    for (size_t j = 1; j <= theta7_samples; j++) {
      std::vector<Vector7> solutions =
          PandaIKFast::inverse(poses_[idx_pose], theta7_values[j]);

      size_t n_sols = solutions.size();
      size_t start = states_.size();
      for (size_t sol_idx = 0; sol_idx < n_sols; ++sol_idx) {
        states_.push_back(solutions[sol_idx]);
        statetopose_.push_back(idx_pose);
        posetostates_[idx_pose].push_back(sol_idx + start);
      }
    }
  }
  lastupdatedpose_ = poses_.size();
}

size_t RoadMap::getPoseFromState(const size_t stateindex) {
  return statetopose_[stateindex];
}

const std::vector<size_t> RoadMap::getStatesFromPose(
    const size_t poseindex) {
  return posetostates_[poseindex];
}

void RoadMap::createGraph() {
  // Iteratively "add" the nodes to the graph. Each time a node is "added",
  // check the distance with the previously "added" nodes. If distance is less
  // than threshold add edge between the nodes
  adjacencylist_ = std::vector<std::vector<RoadMap::Edge>>(states_.size());
  for (size_t i = 0; i < states_.size(); ++i) {
    for (size_t j = 0; j < i; ++j) {
      Vector7 diff = states_[i] - states_[j];
      double distance = diff.norm();
      if (distance < threshold_) {
        adjacencylist_[i].push_back({distance, j});
        adjacencylist_[j].push_back({distance, i});
      }
    }
  }
}

const std::vector<std::vector<RoadMap::Edge>> RoadMap::getadjacencylist() {
  return adjacencylist_;
}

size_t RoadMap::findClosestNodeState(const gtsam::Vector7& states) {
  // For now no weighted distance but may be interesting?

  // Iteritavely go through all stored joint states, compute norm of difference
  // to the given state and check if it is smaller than the minimum one found
  double min_value = 0;
  size_t argmin_node = 0;
  bool first = true;
  for (size_t i = 0; i < states_.size(); ++i) {
    Vector7 diff = states_[i] - states;
    double distance = diff.norm();
    if (first || distance < min_value) {
      argmin_node = i;
      min_value = distance;
      first = false;
    }
  }

  // return the index of the closest joint state node found
  return argmin_node;
}

const std::vector<size_t> RoadMap::findClosestNodesPose(
    const gtsam::Pose3& pose) {
  // only comes from the difference in position, not orientation (as all nodes
  // will have the same)

  Point3 position = pose.translation();

  // Iteritavely go through all stored poses, compute norm of difference to the
  // given pose and check if it is smaller than the minimum one found.
  double min_value = 0;
  size_t argmin_node = 0;
  bool first = true;
  for (size_t i = 0; i < poses_.size(); ++i) {
    Point3 diff = poses_[i].translation() - position;
    double distance = diff.norm();
    if (first || distance < min_value) {
      argmin_node = i;
      min_value = distance;
      first = false;
    }
  }
  // return all node indices with corresponding pose being the closest pose
  // found
  return posetostates_[argmin_node];
}

// Dijkstra's algorithm
std::vector<std::vector<size_t>> RoadMap::findWaypoints(const size_t start_node,
                                                        const size_t end_pose) {
  // this can be changed to limit the number of shortest paths to be found to
  // end_pose before stopping, in usual case 1 because we are finding the
  // shortest among them all
  size_t num_maxpaths = num_maxpaths_;

  size_t num_nodes = adjacencylist_.size();

  // Declare vector where, for each node, the shortest path distance to it from
  // source will be stored and also the corresponding previous node (the parent
  // node in the path)
  std::vector<std::pair<double, size_t>> shortest_path(
      num_nodes, {std::numeric_limits<double>::max(), num_nodes});

  // Initialize source node with 0 distance and arbitrary impossible parent node
  shortest_path[start_node] = {0, num_nodes + 1};

  bool cont = true;
  std::vector<size_t> end_nodes_found;

  // Dijkstra's algorithm:
  // Use priority queue to get the minimum distance node every time
  std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>> p_queue;
  p_queue.push({0, start_node});
  while (cont && not p_queue.empty()) {
    double distance = p_queue.top().first;
    size_t parent_node = p_queue.top().second;
    p_queue.pop();

    // Out of all the possible end_nodes (that have corresponding pose equal to
    // end_pose), store the ones with solution. The ndoes will be ordered in
    // ascending order of path distance
    if (statetopose_[parent_node] == end_pose) {
      end_nodes_found.push_back(parent_node);

      // total number of solutions wanted satisfied
      cont = end_nodes_found.size() < num_maxpaths;
    }

    // check to see if it's the shortest path one (to ease computation)
    if (distance == shortest_path[parent_node].first) {
      // For each edge from parent_node, check if the shortest path from source
      // to parent_node adding edge to a child node is shorter than the shortest
      // path one found previously for the child node
      for (Edge edge : adjacencylist_[parent_node]) {
        double w = edge.first;
        size_t ch = edge.second;
        if (shortest_path[ch].first > shortest_path[parent_node].first + w) {
          shortest_path[ch] = {shortest_path[parent_node].first + w,
                               parent_node};
          p_queue.push({shortest_path[ch].first, ch});
        }
      }
    }
  }

  // Thanks to the priority queue, end_nodes_found is already sorted by distance
  // value in ascending order
  // We need to reconstruct the path
  std::vector<std::vector<size_t>> waypoints(end_nodes_found.size());
  for (size_t i = 0; i < waypoints.size(); ++i) {
    size_t end_node = end_nodes_found[i];
    // We first get the inverse path: start from the end_node and go to the
    // parent node defined in the distance vector
    size_t current_node = end_node;
    while (current_node != start_node) {
      waypoints[i].push_back(current_node);
      current_node = shortest_path[current_node].second;
    }
    waypoints[i].push_back(start_node);

    // Now we need to reverse the order:
    std::reverse(waypoints[i].begin(), waypoints[i].end());
  }

  return waypoints;
}

}  // namespace gtdynamics
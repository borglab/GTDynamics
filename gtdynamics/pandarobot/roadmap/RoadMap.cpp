/**
 * @file  RoadMap.h
 * @brief  Roadmap
 * @author Frank Dellaert
 * @author Antoni Jubes
 */

#include "RoadMap.h"

#include <gtdynamics/pandarobot/ikfast/PandaIKFast.h>
#include <gtsam/geometry/Pose3.h>

#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
#include <string>
#include <tuple>
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
  num_deleted_states_ = 0;
  num_not_found_ = 0;
  for (size_t i = 0; i < 9; ++i) aftercheck_distribution[i] = 0;
  for (size_t i = 0; i < 9; ++i) beforecheck_distribution[i] = 0;
}

RoadMap& RoadMap::addPoseNodes(const std::vector<Pose3>& poses) {
  // concatenate previous poses with given ones
  std::vector<Pose3> concat;
  concat.reserve(poses_.size() + poses.size());  // preallocate memory
  concat.insert(concat.end(), poses_.begin(), poses_.end());
  concat.insert(concat.end(), poses.begin(), poses.end());
  poses_ = concat;

  // add new pose indices to posetostates_
  std::vector<std::vector<size_t>> newposetostates(poses_.size() +
                                                   poses.size());
  newposetostates.insert(newposetostates.begin(), posetostates_.begin(),
                         posetostates_.end());
  posetostates_ = newposetostates;

  return *this;
}

RoadMap& RoadMap::addStateNodes(const std::vector<Vector7>& states) {
  // concatenate previous nodes with given ones
  //! check boundaries missing
  std::vector<Vector7> concat;
  concat.reserve(states_.size() + states.size());  // preallocate memory
  concat.insert(concat.end(), states_.begin(), states_.end());
  concat.insert(concat.end(), states.begin(), states.end());
  states_ = concat;

  return *this;
}

// Return true when within limits
bool RoadMap::checkJointLimits(const Vector7& joint_state) {
  Vector7 lim_inf, lim_sup;
  lim_sup << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
  lim_inf << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
  for (size_t i = 0; i < 7; i++) {
    // Check if value is within boundaries
    if (lim_inf[i] > joint_state[i] || joint_state[i] > lim_sup[i]) {
      if (joint_state[i] < lim_sup[i] - 2 * M_PI) {
        std::cout << "Epp!!! <" << i + 1 << ": " << lim_inf[i] - 2 * M_PI
                  << " < " << joint_state[i] << " < " << lim_sup[i] - 2 * M_PI
                  << std::endl;
      }
      if (joint_state[i] > lim_inf[i] + 2 * M_PI) {
        std::cout << "Epp!!! >" << i + 1 << ": " << lim_inf[i] + 2 * M_PI
                  << " < " << joint_state[i] << " < " << lim_sup[i] + 2 * M_PI
                  << std::endl;
      }
      return false;
    }
  }
  return true;
}

std::vector<double> theta7Discretization(const size_t theta7_samples) {
  double theta7_l_lim = -2.8973, theta7_r_lim = 2.8973;
  double interval = theta7_r_lim - theta7_l_lim;
  std::vector<double> theta7_values(theta7_samples);
  for (size_t i = 0; i < theta7_samples; ++i)
    theta7_values[i] = theta7_l_lim + double((i + 1)) * interval /
                                          double((theta7_samples + 1));

  return theta7_values;
}

/**
 * For each pose (with corresponding pose_index), compute ik solutions with
 * all possible theta values and store:
 *    1) jointindex -> solution : in states_
 *    2) joint_index -> pose_index : in statetopose_
 *    3) pose_index -> [joint_index] : in posetostates_
 * If this is changed, every vector of posetostates_ should still be ordered
 * ascendently by the theta7 value
 */
void RoadMap::computeStateSolutions(const size_t theta7_samples) {
  // Get the possible different values for theta7
  // TODO: factorize this with unittests, choose random or uniform sampling
  theta7_samples_ = theta7_samples;
  std::vector<double> theta7_values = theta7Discretization(theta7_samples);

  for (size_t idx_pose = lastupdatedpose_; idx_pose < poses_.size();
       ++idx_pose) {
    posetostates_[idx_pose] = std::vector<size_t>();

    for (size_t j = 0; j < theta7_samples; j++) {
      std::vector<Vector7> solutions =
          PandaIKFast::inverse(poses_[idx_pose], theta7_values[j]);

      size_t sols_size = solutions.size();
      size_t n_sols_valid = 0;
      if (sols_size == 0) num_not_found_++;
      size_t start = states_.size();
      for (size_t sol_idx = 0; sol_idx < sols_size; ++sol_idx) {
        // change joint 6 domain from [-PI,PI] (default ikfast one) to
        // [-0.5,-0.5+2*PI] so joint limits [-0.0175, 3.7525] are within the
        // domain
        if (solutions[sol_idx](5) < -0.5) {
          solutions[sol_idx](5) += 2 * M_PI;
        }
        // check boundaries and add to counter if deleted
        if (checkJointLimits(solutions[sol_idx])) {
          states_.push_back(solutions[sol_idx]);
          statetopose_.push_back(idx_pose);
          posetostates_[idx_pose].push_back(states_.size() - 1);
          n_sols_valid++;
        } else {
          ++num_deleted_states_;
        }
      }
      if (sols_size <= 8) {
        beforecheck_distribution[sols_size]++;
      } else {
        std::cout << "what??? in computestatesolutions I got more than 8 "
                     "solutions from ikfast at once"
                  << std::endl;
      }
      if (n_sols_valid <= 8) {
        aftercheck_distribution[n_sols_valid]++;
      }
    }
  }
  lastupdatedpose_ = poses_.size();
}

size_t RoadMap::getPoseFromState(const size_t stateindex) {
  return statetopose_[stateindex];
}

const std::vector<size_t> RoadMap::getStatesFromPose(const size_t poseindex) {
  return posetostates_[poseindex];
}

void RoadMap::createGraph() {
  // Iteratively "add" the nodes to the graph. Each time a node is "added",
  // check the distance with the previously "added" nodes. If distance is less
  // than threshold add edge between the nodes
  adjacencylist_ = std::vector<std::vector<RoadMap::Edge>>(states_.size());
  for (size_t i = 0; i < states_.size(); ++i) {
    for (size_t j = 0; j < states_.size(); ++j) {
      if (i != j) {
        Vector7 diff = states_[i] - states_[j];
        double distance = diff.norm();
        if (distance <= threshold_) {
          adjacencylist_[i].push_back({distance, j});
        }
        // if (count%1000==0) std::cout << count/1000 << " i,j: " <<i << ", " <<
        // j
        // << std::endl;
      }
    }
  }
}

std::vector<size_t> indexStatesByTheta(
    const std::vector<size_t>& states_indices,
    const std::vector<gtsam::Vector7>& states_values,
    const size_t theta7_samples) {
  std::vector<double> theta7_values = theta7Discretization(theta7_samples);
  std::vector<size_t> indexbytheta7(theta7_samples);
  size_t theta_idx = 0;
  for (size_t state_idx = 0; state_idx < states_indices.size(); state_idx++) {
    while (theta_idx < theta7_samples &&
           theta7_values[theta_idx] <=
               states_values[states_indices[state_idx]](6)) {
      indexbytheta7[theta_idx] = states_indices[state_idx];
      theta_idx++;
    }
  }
  while (theta_idx < theta7_samples) {
    indexbytheta7[theta_idx] = states_indices.back() + 1;
    theta_idx++;
  }

  return indexbytheta7;
}

// Relationships will be a list of pairs for each state node. These pairs
// represent each a region of Example: list is [(1,4),(8,10)]. This represents:
// [1,4) u [8,10) = {1,2,3,8,9}
// if we had the state nodes indexed by theta7 value too this function would be
// trivial to do, but we don't have it indexed by theta7. Our assumption is that
// the node indices from getStatesFromPose are list of contiguous indices
// that are also ordered ascendengly by theta7 value
std::vector<std::vector<std::pair<size_t, size_t>>>
RoadMap::computeStateLocality(
    const std::vector<std::vector<size_t>>& pose_locality,
    size_t theta_kernel_size) {
  const std::vector<Vector7>& states = getstatenodes();
  std::vector<std::vector<std::pair<size_t, size_t>>> states_locality(
      states.size());

  for (size_t i = 0; i < pose_locality.size(); i++) {
    std::vector<size_t> states_i = getStatesFromPose(i);

    if (states_i.size() > 0) {
      std::vector<size_t> thetaindexed_i =
          indexStatesByTheta(states_i, states, theta7_samples_);

      for (size_t j = 0; j < pose_locality[i].size(); ++j) {
        // For each pose sufficiently close (pose_locality), get its
        // corresponding nodes.
        const std::vector<size_t> states_j =
            getStatesFromPose(pose_locality[i][j]);

        if (states_j.size() > 0) {
          std::vector<size_t> thetaindexed_j =
              indexStatesByTheta(states_j, states, theta7_samples_);

          for (size_t theta_idx = 0; theta_idx < thetaindexed_i.size();
               theta_idx++) {
            size_t state_l_lim = thetaindexed_i[theta_idx];
            size_t state_r_lim =
                (theta_idx < theta7_samples_ - 1 ? thetaindexed_i[theta_idx + 1]
                                                 : states_i.back() + 1);

            size_t theta_l_slider =
                (theta_idx > theta_kernel_size ? theta_idx - theta_kernel_size
                                               : 0);
            size_t theta_r_slider = theta_idx + theta_kernel_size + 1;

            size_t state_l_slider = thetaindexed_j[theta_l_slider];
            size_t state_r_slider = (theta_r_slider < theta7_samples_
                                         ? thetaindexed_j[theta_r_slider]
                                         : states_j.back() + 1);

            if (state_l_slider < state_r_slider) {
              for (size_t idx = state_l_lim; idx < state_r_lim; idx++) {
                states_locality[idx].push_back(
                    {state_l_slider, state_r_slider});
              }
            }
          }
        }
      }
    }
  }

  return states_locality;
}

void RoadMap::createGraphFromReference(
    const std::vector<std::vector<std::pair<size_t, size_t>>>& reference) {
  adjacencylist_ = std::vector<std::vector<RoadMap::Edge>>(states_.size());
  for (size_t i = 0; i < reference.size(); i++) {
    for (size_t j = 0; j < reference[i].size(); ++j) {
      for (size_t idx = reference[i][j].first; idx < reference[i][j].second;
           ++idx) {
        if (i != idx) {
          Vector7 diff = states_[i] - states_[idx];
          double distance = diff.norm();
          if (distance <= threshold_) {
            adjacencylist_[i].push_back({distance, idx});
          }
        }
      }
    }
  }
}

/*
void RoadMap::createGraphFromReference(
    const std::vector<std::vector<size_t>>& relationships) {
  adjacencylist_ = std::vector<std::vector<RoadMap::Edge>>(states_.size());
  for (size_t i = 0; i < relationships.size(); i++) {
    for (size_t j = 0; j < relationships[i].size(); ++j) {
      if (i != relationships[i][j]) {
        Vector7 diff = states_[i] - states_[relationships[i][j]];
        double distance = diff.norm();
        if (distance <= threshold_) {
          adjacencylist_[i].push_back({distance, relationships[i][j]});
        }
      }
    }
  }
}
*/

const std::vector<std::vector<RoadMap::Edge>>& RoadMap::getadjacencylist()
    const {
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
  return this->getStatesFromPose(argmin_node);
}
/*
struct Node {
  size_t level, idx;
  Node(size_t l, size_t i) : level{l}, idx{i} {}
};*/

void Heuristic::preprocess(const RoadMap& roadmap,
                           const std::vector<std::vector<size_t>>& waypoints) {}
double Heuristic::operator()(const RoadMap& roadmap, const Node& node) {
  return 0;
}

void DirectDistance::preprocess(
    const RoadMap& roadmap, const std::vector<std::vector<size_t>>& waypoints) {
  end_nodes = waypoints.back();
}
double DirectDistance::operator()(const RoadMap& roadmap, const Node& node) {
  double h = std::numeric_limits<double>::max();
  for (size_t i = 0; i < end_nodes.size(); i++) {
    double h_temp = (roadmap.getstatenodes()[node.idx] -
                     roadmap.getstatenodes()[end_nodes[i]])
                        .norm();
    h = h_temp < h ? h_temp : h;
  }

  return h;
}

// this function is only valid for a consistent Heuristic
// basic dijsktra is h=0
// A* with basic h being h(n_ij, goal) = min(h(waypointset_j,goal) +
// h(n_ij,waypointset_j)) for all points in waypointset_j, where
// h(n_ij,waypointset_j) is just min(d(n_ij,waypointset_jk)) for all k
// Cost for each point: len(waypointset_jk)*(cost to compute distance)
// A* with landmarks will reduce the cost, but h might be a lower lowerbound
// have to continue reading the paper
std::vector<std::vector<size_t>> RoadMap::findPath(
    const std::vector<std::vector<size_t>>& waypoints, Heuristic* h,
    const std::string& save_path) {
  h->preprocess(*this, waypoints);
  size_t num_nodes = adjacencylist_.size();

  // Declare map from graph node -> waypoint set . If it doesn't correspond to
  // any waypoint, the value is waypoints.size()
  std::vector<size_t> waypointset(num_nodes, waypoints.size());
  for (size_t i = 0; i < waypoints.size(); i++) {
    for (size_t j = 0; j < waypoints[i].size(); j++) {
      waypointset[waypoints[i][j]] = i;
    }
  }

  // Declare vector where, for each node, the shortest path distance to it from
  // source will be stored and also the corresponding previous node (the parent
  // node in the path)
  std::vector<std::vector<std::pair<double, Node>>> shortest_path(
      waypoints.size() - 1,
      std::vector<std::pair<double, Node>>(
          num_nodes, {std::numeric_limits<double>::max(),
                      Node({waypoints.size() - 1, num_nodes})}));

  // Initialize all source nodes with 0 distance and arbitrary impossible parent
  // node
  for (size_t i = 0; i < waypoints[0].size(); i++) {
    shortest_path[0][waypoints[0][i]] = {0, Node({0, num_nodes + 1})};
  }

  bool cont = true;
  std::vector<size_t> end_nodes_found;

  struct WaypointEdge {
    double f;
    double g;
    Node parent;
    bool operator>(const WaypointEdge& r) const {
      if (this->f == r.f) {
        return this->g < r.g;
      }
      return this->f > r.g;
    }
  };

  // A* algorithm:
  // Use priority queue to get the minimum distance node every time
  std::priority_queue<WaypointEdge, std::vector<WaypointEdge>,
                      std::greater<WaypointEdge>>
      p_queue;

  // Push into queue the start nodes
  for (size_t i = 0; i < waypoints[0].size(); i++) {
    p_queue.push({h->operator()(*this, Node({0, waypoints[0][i]})), 0,
                  Node({0, waypoints[0][i]})});
  }

  while (cont && not p_queue.empty()) {
    WaypointEdge top = p_queue.top();
    Node parent = top.parent;
    p_queue.pop();

    // check to see that it's not the
    if (top.g == shortest_path[parent.level][parent.idx].first) {
      if (waypointset[parent.idx] == parent.level + 1) {
        // Out of all the possible end_nodes (that have corresponding pose equal
        // to end_pose), store the ones with solution. The ndoes will be ordered
        // in ascending order of path distance
        if (waypointset[parent.idx] == waypoints.size() - 1) {
          // Save goal node
          end_nodes_found.push_back(parent.idx);

          // total number of solutions wanted satisfied
          cont = end_nodes_found.size() < num_maxpaths_;
        } else {
          // change level
          shortest_path[parent.level + 1][parent.idx] = {
              shortest_path[parent.level][parent.idx].first, parent};
          p_queue.push({top.f, top.g, Node({parent.level + 1, parent.idx})});
        }
      } else {
        // For each edge from parent, check if the shortest path from source
        // to parent adding edge to a child node is shorter than the shortest
        // path one found previously for the child node
        for (Edge edge : adjacencylist_[parent.idx]) {
          double w = edge.first;
          size_t ch = edge.second;
          if (top.g + w < shortest_path[parent.level][ch].first) {
            shortest_path[parent.level][ch] = {
                shortest_path[parent.level][parent.idx].first + w, parent};
            p_queue.push(
                {top.g + w + h->operator()(*this, Node({parent.level, ch})),
                 top.g + w, Node({parent.level, ch})});
          }
        }
      }
    }
  }
  std::cout << "cont: " << (cont ? "yes" : "no") << std::endl;
  std::cout << "queue empty? " << (p_queue.empty() ? "yes" : "no") << std::endl;

  // Thanks to the priority queue, end_nodes_found is already sorted by
  // distance value in ascending order We need to reconstruct the path
  std::vector<std::vector<size_t>> paths(end_nodes_found.size());
  for (size_t i = 0; i < paths.size(); ++i) {
    size_t goal_node = end_nodes_found[i];
    // We first get the inverse path: start from the end_node and go to the
    // parent node defined in the distance vector
    Node current_node = {waypoints.size() - 2, goal_node};
    size_t counter = 0;
    while (waypointset[current_node.idx] != 0 && counter++ < 1000) {
      paths[i].push_back(current_node.idx);
      current_node = shortest_path[current_node.level][current_node.idx].second;
      if (current_node.idx == paths[i].back()) {
        current_node =
            shortest_path[current_node.level][current_node.idx].second;
      }
    }
    paths[i].push_back(current_node.idx);

    // Now we need to reverse the order:
    std::reverse(paths[i].begin(), paths[i].end());
  }

  // change this!!!!!
  if (not save_path.empty()) {
    std::ofstream output(save_path);

    output << "--poseindices::\n";
    for (size_t i = 0; i < statetopose_.size(); ++i) {
      output << statetopose_[i] << " ";
    }

    output << "\n--stagepoints::\n";
    for (size_t i = 0; i < waypoints.size(); i++) {
      output << i << ":";
      for (const size_t& x : waypoints[i]) output << " " << x;
      output << "\n";
    }

    output << "--paths::\n";
    for (size_t i = 0; i < paths.size(); i++) {
      output << i << ":";
      for (size_t& x : paths[i]) output << " " << x;
      output << "\n";
    }

    output << "--data::\n";
    for (size_t i = 0; i < shortest_path.size(); i++) {
      for (size_t j = 0; j < num_nodes; j++) {
        output << shortest_path[i][j].first << " ";
      }
      output << "\n";
    }
    output.close();
  }

  return paths;
}

// Dijkstra's algorithm
std::vector<std::vector<size_t>> RoadMap::findWaypoints(const size_t start_node,
                                                        const size_t end_pose) {
  // this can be changed to limit the number of shortest paths to be found to
  // end_pose before stopping, in usual case 1 because we are finding the
  // shortest among them all
  size_t num_maxpaths = num_maxpaths_;

  size_t num_nodes = adjacencylist_.size();

  std::vector<std::vector<size_t>> waypoints_to_follow(2);
  waypoints_to_follow[0] = {start_node};
  waypoints_to_follow[1] = this->getStatesFromPose(end_pose);
  Heuristic h;

  std::vector<std::vector<size_t>> paths = findPath(waypoints_to_follow, &h);

  return paths;
}

}  // namespace gtdynamics
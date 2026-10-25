/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file ForwardKinematics.h
 * @brief Forward kinematics for the universal robot.
 * @author: Varun Agrawal
 */

#pragma once

#include <gtdynamics/universal_robot/Joint.h>
#include <gtdynamics/universal_robot/Link.h>
#include <gtsam/nonlinear/Values.h>

#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace gtdynamics {

/**
 * Breadth-first traversal of the link-joint graph from a root link, which
 * propagates poses and twists from the root to every other link.
 */
class GTSAM_EXPORT FKTraversal {
  /// A traversed joint, as indices into links_.
  struct Edge {
    JointSharedPtr joint;
    size_t from, to;
  };

  std::vector<LinkSharedPtr> links_;  ///< BFS order, links_[0] is the root.
  std::vector<Edge> tree_edges_;      ///< Spanning tree joints, in BFS order.
  std::vector<Edge> loop_edges_;      ///< Joints that close kinematic loops.
  std::vector<size_t> versions_;      ///< Topology version of each link, used
                                      ///< to detect topology changes.

 public:
  /// Compute the traversal of the link-joint graph starting at `root`.
  explicit FKTraversal(const LinkSharedPtr &root);

  /// Check the traversal still matches the joints attached to its links.
  bool isCurrent() const;

  /**
   * Compute the pose and twist of every link reachable from the root, whose
   * pose and twist must already be in `values`. Missing joint angles and
   * velocities default to zero. Poses and twists already in `values` are
   * checked for consistency (throwing std::runtime_error if inconsistent) and
   * not propagated further.
   */
  void run(size_t t, gtsam::Values *values) const;
};

/**
 * Thread-safe cache of forward kinematics traversals, keyed by root link.
 * Traversals are computed lazily, the first time a root is requested, and
 * recomputed if joints were added to or removed from any of their links.
 */
class GTSAM_EXPORT FKTraversalCache {
  mutable std::mutex mutex_;
  std::unordered_map<const Link *, std::shared_ptr<const FKTraversal>>
      traversals_;

 public:
  FKTraversalCache() = default;

  /// Copy the cached traversals, keeping an independent mutex.
  FKTraversalCache(const FKTraversalCache &other);
  FKTraversalCache &operator=(const FKTraversalCache &other);

  /// Move the cached traversals, keeping an independent mutex.
  FKTraversalCache(FKTraversalCache &&other);
  FKTraversalCache &operator=(FKTraversalCache &&other);

  /// Return the traversal rooted at `root`, computing it if missing or stale.
  std::shared_ptr<const FKTraversal> get(const LinkSharedPtr &root);

  /// Discard all cached traversals.
  void clear();
};

}  // namespace gtdynamics

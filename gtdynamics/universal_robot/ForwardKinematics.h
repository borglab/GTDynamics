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
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace gtdynamics {

/// A joint traversed in forward kinematics, as indices into
/// FKTraversal::links.
struct FKEdge {
  JointSharedPtr joint;
  size_t from, to;
};

/// Breadth-first traversal of the link-joint graph from a root link.
struct FKTraversal {
  std::vector<LinkSharedPtr> links;  ///< BFS order, links[0] is the root.
  std::vector<FKEdge> tree_edges;    ///< Spanning tree joints, in BFS order.
  std::vector<FKEdge> loop_edges;    ///< Joints that close kinematic loops.
  std::vector<size_t> num_joints;    ///< Joint count of each link, used to
                                     ///< detect topology changes.

  /// Check the traversal still matches the joints attached to its links.
  bool isCurrent() const;
};

/// Map from root link to its forward kinematics traversal.
using FKTraversalMap =
    std::unordered_map<const Link *, std::shared_ptr<const FKTraversal>>;

/**
 * Thread-safe cache of forward kinematics traversals, keyed by root link.
 * Traversals are computed lazily, the first time a root is requested, and
 * recomputed if the joints attached to their links have changed.
 */
class FKTraversalCache {
  std::mutex mutex_;
  FKTraversalMap traversals_;

 public:
  /// Return the traversal rooted at `root`, computing it if missing or stale.
  std::shared_ptr<const FKTraversal> get(const LinkSharedPtr &root);
};

// map from link name to link pose
using LinkPoses = std::map<std::string, gtsam::Pose3>;
// map from link name to link twist
using LinkTwists = std::map<std::string, gtsam::Vector6>;
// type for storing forward kinematics results
using FKResults = std::pair<LinkPoses, LinkTwists>;

/// Compute the forward kinematics traversal starting at `root`.
FKTraversal ComputeFKTraversal(const LinkSharedPtr &root);

/**
 * Compute pose/twist of the link on the other side of `joint`,
 * insert them into values (or check consistency if already present)
 * and return whether they were inserted.
 */
bool Propagate(const JointSharedPtr &joint, const LinkSharedPtr &link1,
               const gtsam::Pose3 &T_w1, const gtsam::Vector6 &V_1, size_t t,
               gtsam::Values *values,
               std::pair<gtsam::Pose3, gtsam::Vector6> *poseTwist);

/// Forward kinematics by BFS, discovering the traversal order on the fly.
void BreadthFirstKinematics(const LinkSharedPtr &root_link, size_t t,
                            gtsam::Values *values);

}  // namespace gtdynamics

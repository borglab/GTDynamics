/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file DynamicsGraph.h
 * @brief Builds a lean dynamics graph from a Robot object.
 * @author Dan Barladeanu
 */

#include "gtdynamics/dynamics/LeanDynamicsGraph.h"

namespace gtdynamics {

gtsam::NonlinearFactorGraph LeanDynamicsGraph::chainFactors(const int t) const {

  gtsam::NonlinearFactorGraph graph;

  for (int i = 0; i < composed_chains_.size() ; ++i) {
        // Get id of hip joint on this chain
        uint8_t hip_joint_id = chain_joints_[i][0]->id();

        // Get key for wrench at joint hip_joint_id on link 0 at time t
        const gtsam::Key wrench_key = gtdynamics::WrenchKey(0, hip_joint_id, t);

        // Create expression 
        auto expression = const_cast<Chain&>(composed_chains_[i]).ChainConstraint3(chain_joints_[i], wrench_key, t);

        // Create VectorExpressionEquality Constraint
        auto constraint = VectorExpressionEquality<3>(expression, tolerance_);

        // Create factor
        auto factor = constraint.createFactor(1.0);

        graph.add(factor);
      }

  return graph;
}

gtsam::NonlinearFactorGraph LeanDynamicsGraph::dynamicsFactorGraph(
    const Robot &robot, const int t,
    const boost::optional<PointOnLinks> &contact_points,
    const boost::optional<double> &mu) const {

      gtsam::NonlinearFactorGraph graph;

      graph.add(qFactors(robot, t, contact_points));
      //graph.add(vFactors(robot, t, contact_points));
      //graph.add(aFactors(robot, t, contact_points));
      graph.add(dynamicsFactors(robot, t, contact_points, mu));          

      graph.add(chainFactors(t));

      return graph;
    }
}
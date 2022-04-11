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
#include "gtdynamics/factors/WrenchFactor.h"
#include "gtdynamics/factors/ContactDynamicsFrictionConeFactor.h"
#include "gtdynamics/factors/ContactDynamicsMomentFactor.h"
#include "gtdynamics/factors/TorqueFactor.h"
#include "gtdynamics/factors/WrenchEquivalenceFactor.h"

namespace gtdynamics {

gtsam::NonlinearFactorGraph LeanDynamicsGraph::chainFactors(
    const Robot &robot, const int t,
    const boost::optional<PointOnLinks> &contact_points) const {
  gtsam::NonlinearFactorGraph graph;

  std::vector<DynamicsSymbol> wrench_keys;

  for (int i = 0; i < composed_chains_.size() ; ++i) {
        // Get id of hip joint on this chain
        uint8_t hip_joint_id = chain_joints_[i][0]->id();

        // Get key for wrench at joint hip_joint_id on link 0 at time t
        const gtsam::Key wrench_key = gtdynamics::WrenchKey(0, hip_joint_id, t);

        // add wrench to wrench vector
        wrench_keys.push_back(wrench_key);

        // Create expression 
        auto expression = const_cast<Chain&>(composed_chains_[i]).ChainConstraint3(chain_joints_[i], wrench_key, t);

        // Create VectorExpressionEquality Constraint
        auto constraint = VectorExpressionEquality<3>(expression, tolerance_);

        // Create factor
        auto factor = constraint.createFactor(1.0);

        graph.add(factor);
  }

    /*double mu_ = 1;
    if (contact_points) {
      int c = 0;
      for (auto &&cp : *contact_points) {
        //if (cp.link->name() != i) continue;
          // TODO(frank): allow multiple contact points on one link, id = 0,1,..
          auto wrench_key = ContactWrenchKey(0, c++, t);
          wrench_keys.push_back(wrench_key);

          // Add contact dynamics constraints.
          graph.emplace_shared<ContactDynamicsFrictionConeFactor>(
              PoseKey(0, t), wrench_key, opt().cfriction_cost_model, mu_,
             *gravity());

          graph.emplace_shared<ContactDynamicsMomentFactor>(
              wrench_key, opt().cm_cost_model,
              gtsam::Pose3(gtsam::Rot3(), -cp.point));
      }
    }*/

      // add wrench factor for link
  graph.add(
    WrenchFactor(opt().fa_cost_model, robot.link("trunk"), wrench_keys, t, gravity()));

  for (auto &&link : robot.links()) {
    int i = link->id();
    if (i==0) continue;
    if (!link->isFixed()) {
      const auto &connected_joints = link->joints();
      std::vector<DynamicsSymbol> wrench_keys;

      // Add wrench keys for joints.
      for (auto &&joint : connected_joints)
        wrench_keys.push_back(WrenchKey(i, joint->id(), t));

      // add wrench factor for link
      graph.add(
          WrenchFactor(opt().fa_cost_model, link, wrench_keys, t, gravity()));
    }
  }

  for (auto &&joint : robot.joints()) {
    auto j = joint->id(), child_id = joint->child()->id();
    auto const_joint = joint;
    graph.add(WrenchEquivalenceFactor(opt().f_cost_model, const_joint, t));
    graph.add(TorqueFactor(opt().t_cost_model, const_joint, t));
  }

  return graph;
}

gtsam::NonlinearFactorGraph LeanDynamicsGraph::dynamicsFactorGraph(
    const Robot &robot, const int t,
    const boost::optional<PointOnLinks> &contact_points,
    const boost::optional<double> &mu) const {

      gtsam::NonlinearFactorGraph graph;

      graph.add(qFactors(robot, t, contact_points));
      graph.add(vFactors(robot, t, contact_points));
      graph.add(aFactors(robot, t, contact_points));
      //graph.add(dynamicsFactors(robot, t, contact_points, mu));          
      graph.add(chainFactors(robot, t, contact_points));

      return graph;
    }
}
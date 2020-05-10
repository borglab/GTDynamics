/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearDynamicsEliminateableGraph.h
 * @brief Builds an eliminateable nonlinear dynamics graph from a Robot object.
 * @author Mandy Xie
 */

#pragma once

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/dynamics/OptimizerSetting.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtsam/inference/EliminateableFactorGraph.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <boost/shared_ptr.hpp>
#include <functional>

namespace gtdynamics {
// Forward declarations
class NonlinearDynamicsEliminateableGraph;
class NonlinearDynamicsConditional;
class NonlinearDynamicsBayesNet;
class NonlinearDynamicsEliminationTree;
class NonlinearDynamicsBayesTree;
class NonlinearDynamicsJunctionTree;
}  // namespace gtdynamics

/** Main elimination function for NonlinearDynamicsEliminateableGraph */
std::pair<boost::shared_ptr<gtdynamics::NonlinearDynamicsConditional>,
          gtsam::NonlinearFactor::shared_ptr>
EliminateNonlinear(
    const gtdynamics::NonlinearDynamicsEliminateableGraph& factors,
    const gtsam::Ordering& keys);
/* ************************************************************************* */
template <>
struct gtsam::EliminationTraits<
    gtdynamics::NonlinearDynamicsEliminateableGraph> {
  typedef gtsam::NonlinearFactor
      FactorType;  ///< Type of factors in factor graph
  typedef gtdynamics::NonlinearDynamicsEliminateableGraph
      FactorGraphType;  ///< Type of the factor graph
                        ///(e.g. NonlinearDynamicsEliminateableGraph)
  typedef gtdynamics::NonlinearDynamicsConditional
      ConditionalType;  ///< Type of conditionals from elimination
  typedef gtdynamics::NonlinearDynamicsBayesNet
      BayesNetType;  ///< Type of Bayes net from sequential elimination
  typedef gtdynamics::NonlinearDynamicsEliminationTree
      EliminationTreeType;  ///< Type of elimination tree
  typedef gtdynamics::NonlinearDynamicsBayesTree BayesTreeType;  ///< Type of Bayes tree
  typedef gtdynamics::NonlinearDynamicsJunctionTree
      JunctionTreeType;  ///< Type of Junction tree
  /// The default dense elimination function
  static std::pair<boost::shared_ptr<ConditionalType>,
                   boost::shared_ptr<FactorType> >
  DefaultEliminate(const FactorGraphType& factors,
                   const gtsam::Ordering& keys) {
    return EliminateNonlinear(factors, keys);
  }
};

namespace gtdynamics {
/**
 * A non-linear eliminateble dynamics graph is a graph of dynamics
 * where we can perform non-linear elimination.
 */
class NonlinearDynamicsEliminateableGraph
    : public gtsam::NonlinearFactorGraph,
      public gtsam::EliminateableFactorGraph<
          NonlinearDynamicsEliminateableGraph> {
 public:
  typedef NonlinearDynamicsEliminateableGraph This;  ///< Typedef to this class
  typedef gtsam::NonlinearFactorGraph
      Base;  ///< Typedef to base factor graph type
  typedef gtsam::EliminateableFactorGraph<This>
      BaseEliminateable;  ///< Typedef to base elimination class
  typedef boost::shared_ptr<This> shared_ptr;  ///< shared_ptr to this class

  /** Default constructor */
  NonlinearDynamicsEliminateableGraph() {}

  /** Construct from NonlinearFactorGraph */
  explicit NonlinearDynamicsEliminateableGraph(
      const gtsam::NonlinearFactorGraph& factors)
      : Base(factors) {}

};  // \ NonlinearDynamicsEliminateableGraph

}  // namespace gtdynamics

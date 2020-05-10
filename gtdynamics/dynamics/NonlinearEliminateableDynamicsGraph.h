/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearEliminateableDynamicsGraph.h
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
class NonlinearEliminateableDynamicsGraph;
class NonlinearConditional;
class NonlinearBayesNet;
class NonlinearEliminationTree;
class NonlinearBayesTree;
class NonlinearJunctionTree;
}  // namespace gtdynamics

/** Main elimination function for NonlinearEliminateableDynamicsGraph */
std::pair<boost::shared_ptr<gtdynamics::NonlinearConditional>,
          gtsam::NonlinearFactor::shared_ptr>
EliminateNonlinear(
    const gtdynamics::NonlinearEliminateableDynamicsGraph& factors,
    const gtsam::Ordering& keys);
/* ************************************************************************* */
template <>
struct gtsam::EliminationTraits<
    gtdynamics::NonlinearEliminateableDynamicsGraph> {
  typedef gtsam::NonlinearFactor
      FactorType;  ///< Type of factors in factor graph
  typedef gtdynamics::NonlinearEliminateableDynamicsGraph
      FactorGraphType;  ///< Type of the factor graph
                        ///(e.g. NonlinearEliminateableDynamicsGraph)
  typedef gtdynamics::NonlinearConditional
      ConditionalType;  ///< Type of conditionals from elimination
  typedef gtdynamics::NonlinearBayesNet
      BayesNetType;  ///< Type of Bayes net from sequential elimination
  typedef gtdynamics::NonlinearEliminationTree
      EliminationTreeType;  ///< Type of elimination tree
  typedef gtdynamics::NonlinearBayesTree BayesTreeType;  ///< Type of Bayes tree
  typedef gtdynamics::NonlinearJunctionTree
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
class NonlinearEliminateableDynamicsGraph
    : public gtsam::NonlinearFactorGraph,
      public gtsam::EliminateableFactorGraph<
          NonlinearEliminateableDynamicsGraph> {
 public:
  typedef NonlinearEliminateableDynamicsGraph This;  ///< Typedef to this class
  typedef gtsam::NonlinearFactorGraph
      Base;  ///< Typedef to base factor graph type
  typedef gtsam::EliminateableFactorGraph<This>
      BaseEliminateable;  ///< Typedef to base elimination class
  typedef boost::shared_ptr<This> shared_ptr;  ///< shared_ptr to this class

  /** Default constructor */
  NonlinearEliminateableDynamicsGraph() {}

  /** Construct from NonlinearFactorGraph */
  explicit NonlinearEliminateableDynamicsGraph(
      const gtsam::NonlinearFactorGraph& factors)
      : Base(factors) {}

};  // \ NonlinearEliminateableDynamicsGraph

}  // namespace gtdynamics

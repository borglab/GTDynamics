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
#include <gtdynamics/factors/TorqueFactor.h>
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

/** Main elimination function for NonlinearDynamicsEliminateableGraph */
std::pair<boost::shared_ptr<NonlinearDynamicsConditional>,
          boost::shared_ptr<TorqueFactor>>
EliminateNonlinear(const NonlinearDynamicsEliminateableGraph& factors,
                   const gtsam::Ordering& keys);
}  // namespace gtdynamics

/* ************************************************************************* */
template <>
struct gtsam::EliminationTraits<
    gtdynamics::NonlinearDynamicsEliminateableGraph> {
  typedef gtdynamics::TorqueFactor
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
  typedef gtdynamics::NonlinearDynamicsBayesTree
      BayesTreeType;  ///< Type of Bayes tree
  typedef gtdynamics::NonlinearDynamicsJunctionTree
      JunctionTreeType;  ///< Type of Junction tree
  /// The default dense elimination function
  static std::pair<boost::shared_ptr<ConditionalType>,
                   boost::shared_ptr<FactorType>>
  DefaultEliminate(const FactorGraphType& factors,
                   const gtsam::Ordering& keys) {
    return gtdynamics::EliminateNonlinear(factors, keys);
  }
};

namespace gtdynamics {
/**
 * A non-linear eliminateble dynamics graph is a graph of dynamics
 * where we can perform non-linear elimination.
 */
class NonlinearDynamicsEliminateableGraph
    : public gtsam::FactorGraph<TorqueFactor>,
      public gtsam::EliminateableFactorGraph<
          NonlinearDynamicsEliminateableGraph> {
 public:
  typedef NonlinearDynamicsEliminateableGraph This;  ///< Typedef to this class
  typedef gtsam::FactorGraph<TorqueFactor>
      Base;  ///< Typedef to base factor graph type
  typedef gtsam::EliminateableFactorGraph<This>
      BaseEliminateable;  ///< Typedef to base elimination class
  typedef boost::shared_ptr<This> shared_ptr;  ///< shared_ptr to this class

  /** Default constructor */
  NonlinearDynamicsEliminateableGraph() {}

  /** Construct from iterator over factors */
  template <typename ITERATOR>
  NonlinearDynamicsEliminateableGraph(ITERATOR firstFactor, ITERATOR lastFactor)
      : Base(firstFactor, lastFactor) {}

  /** Construct from container of factors (shared_ptr or plain objects) */
  template <class CONTAINER>
  explicit NonlinearDynamicsEliminateableGraph(const CONTAINER& factors)
      : Base(factors) {}

  /** Implicit copy/downcast constructor to override explicit template container
   * constructor */
  template <class DERIVEDFACTOR>
  NonlinearDynamicsEliminateableGraph(const FactorGraph<DERIVEDFACTOR>& graph)
      : Base(graph) {}

  /// @name Testable
  /// @{
  bool equals(const This& fg, double tol = 1e-9) const;
  /// @}

};  // \ NonlinearDynamicsEliminateableGraph

}  // namespace gtdynamics

/// traits
template <>
struct gtsam::traits<gtdynamics::NonlinearDynamicsEliminateableGraph>
    : public gtsam::Testable<gtdynamics::NonlinearDynamicsEliminateableGraph> {
};
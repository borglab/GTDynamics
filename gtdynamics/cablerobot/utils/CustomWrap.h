/**
 * @file  CustomWrap.h
 * @brief Custom wrap utilities
 * @author Frank Dellaert
 * @author Gerry Chen
 */

#pragma once

#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtdynamics/utils/values.h>

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/nonlinear/Values.h>

namespace gtdynamics {

using BlockOrdering = std::vector<gtsam::Ordering>;

/// Performs sequential elimination and preserves correct bayes net order
gtsam::GaussianBayesNet::shared_ptr EliminateSequential(
    gtsam::GaussianFactorGraph graph, const gtsam::Ordering& ordering);

/// Performs sequential block elimination and preserves correct bayes net order
gtsam::GaussianBayesNet::shared_ptr BlockEliminateSequential(
    gtsam::GaussianFactorGraph graph,
    const BlockOrdering& ordering);

///@name TensionKey
///@{

/// Shorthand for t_j_k, for j-th tension at time step k.
inline DynamicsSymbol TensionKey(int j, int k = 0) {
  return DynamicsSymbol::JointSymbol("t", j, k);
}

/**
 * @brief Insert j-th tension at time k.
 *
 * @param values Values pointer to insert tension into.
 * @param j The joint id.
 * @param k Time step.
 * @param value The tension value.
 */
void InsertTension(gtsam::Values *values, int j, int k, double value) {
  values->insert(TensionKey(j, k), value);
}

/**
 * @brief Insert j-th tension at time 0.
 *
 * @param values Values pointer to insert tension into.
 * @param j The joint id.
 * @param value The tension value.
 */
void InsertTension(gtsam::Values *values, int j, double value) {
  values->insert(TensionKey(j), value);
}

/**
 * @brief Retrieve j-th tension at time k.
 *
 * @param values Values dictionary containing the tension.
 * @param j The joint id.
 * @param k Time step.
 * @return The tension.
 */
double Tension(const gtsam::Values &values, int j, int k = 0) {
  return at<double>(values, TensionKey(j, k));
}
///@}

}  // namespace gtdynamics

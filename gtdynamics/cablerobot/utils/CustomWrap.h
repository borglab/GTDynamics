/**
 * @file  CustomWrap.h
 * @brief Custom wrap utilities
 * @author Frank Dellaert
 * @author Gerry Chen
 */

#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianConditional.h>

namespace gtdynamics {

using BlockOrdering = std::vector<gtsam::Ordering>;

/// Performs sequential elimination and preserves correct bayes net order
gtsam::GaussianBayesNet::shared_ptr EliminateSequential(
    gtsam::GaussianFactorGraph graph, const gtsam::Ordering& ordering);

/// Performs sequential block elimination and preserves correct bayes net order
gtsam::GaussianBayesNet::shared_ptr BlockEliminateSequential(
    gtsam::GaussianFactorGraph graph,
    const BlockOrdering& ordering);

}  // namespace gtdynamics

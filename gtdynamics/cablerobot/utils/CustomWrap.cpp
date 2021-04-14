/**
 * @file  CustomWrap.cpp
 * @brief Custom wrap utilities
 * @author Frank Dellaert
 * @author Gerry Chen
 */

#include <gtdynamics/cablerobot/utils/CustomWrap.h>

#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianFactorGraph.h>

#include <boost/assign/list_of.hpp>

using namespace gtsam;

namespace gtdynamics {

/// Performs sequential elimination and preserves correct bayes net order
GaussianBayesNet::shared_ptr EliminateSequential(GaussianFactorGraph graph,
                                                 const Ordering& ordering) {
  BlockOrdering blockOrdering;
  for (auto k : ordering)
    blockOrdering.push_back(boost::assign::list_of(k));
  return BlockEliminateSequential(graph, blockOrdering);
}

/// Performs sequential elimination and preserves correct bayes net order
GaussianBayesNet::shared_ptr BlockEliminateSequential(
    GaussianFactorGraph graph, const BlockOrdering &ordering) {
  // setup
  VariableIndex variableIndex(graph);  // maps keys to factor indices
  auto bn = boost::make_shared<GaussianBayesNet>();

  // loop
  for (auto keys : ordering) {
    // collect factors
    GaussianFactorGraph factors;
    for (auto key : keys)
      for (size_t factorindex : variableIndex[key]) {
        factors.push_back(graph.at(factorindex));
        graph.remove(factorindex);
      }
    // eliminate
    auto [conditional, newfactor] =
        EliminationTraits<GaussianFactorGraph>::DefaultEliminate(factors, keys);
    bn->push_back(conditional);
    // add new joint factor
    graph.push_back(newfactor);
    variableIndex.augment(GaussianFactorGraph(newfactor));
  }
  return bn;
}

}  // namespace gtdynamics

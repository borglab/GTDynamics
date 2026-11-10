/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ManifoldOptimizer.cpp
 * @brief Implementation of manifold optimizer.
 * @author: Yetong Zhang
 */

#include <gtdynamics/cmopt/ManifoldOptimizer.h>
#include <gtdynamics/constrained/CMCostFactor.h>
#include <gtdynamics/cmcopt/IEOptimizer.h>
#include <gtdynamics/cmopt/UnconstrainedOptimizationRetractor.h>
#include <gtdynamics/utils/GraphUtils.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace gtdynamics {

/* ************************************************************************* */
Values ManifoldOptimizationProblem::unconstrainedValues() const {
  return SubValues(values, unconstrainedKeys);
}

/* ************************************************************************* */
ECMPoints ManifoldOptimizationProblem::points() const {
  ECMPoints result;
  for (const Key& key : manifoldKeys) {
    result.emplace(key, values.at<ECMPoint>(key));
  }
  return result;
}

/* ************************************************************************* */
ECMPoints ManifoldOptimizationProblem::constPoints() const {
  return fixedPoints;
}

/* ************************************************************************* */
std::pair<size_t, size_t> ManifoldOptimizationProblem::problemDimension() const {
  size_t graph_dim = 0;
  for (const auto& factor : graph) {
    graph_dim += factor->dim();
  }
  return std::make_pair(graph_dim, values.dim());
}

/* ************************************************************************* */
void ManifoldOptimizationProblem::print(const std::string& s,
                               const KeyFormatter& keyFormatter) const {
  std::cout << s;
  std::cout << "found " << types.size()
            << " types: " << manifoldKeys.size() << " free points, "
            << fixedPoints.size() << " fixed points\n";
  for (const Key& key : manifoldKeys) {
    const auto& point = values.at<ECMPoint>(key);
    std::cout << "component " << keyFormatter(key) << ":\tdimension "
              << point.dim() << "\n";
    for (const auto& ambientKey : point.values().keys()) {
      std::cout << "\t" << keyFormatter(ambientKey);
    }
    std::cout << "\n";
  }
  std::cout << "fully constrained points:\n";
  for (const auto& entry : fixedPoints) {
    for (const auto& key : entry.second.values().keys()) {
      std::cout << "\t" << keyFormatter(key);
    }
    std::cout << "\n";
  }
}

/* ************************************************************************* */
ManifoldOptimizerParameters::ManifoldOptimizerParameters()
    : Base(),
      constraintManifoldParams(std::make_shared<EConstraintManifold::Params>()),
      retractInitialValues(true) {
  constraintManifoldParams->retractorCreator =
      std::make_shared<UnconstrainedOptimizationRetractorCreator>();
}

/* ************************************************************************* */
NonlinearFactorGraph ManifoldOptimizer::manifoldGraph(
    const NonlinearFactorGraph& graph, const ECMPoints& points) {
  Values allPoints;
  for (const auto& entry : points) {
    allPoints.insert(entry.first, entry.second);
  }
  return pointCostGraph(graph, allPoints,
                       CMCostFactor::PointType::Equality);
}

/* ************************************************************************* */
ManifoldOptimizationProblem ManifoldOptimizer::transformProblem(
    const EConsOptProblem& equalityConstrainedProblem) const {
  ManifoldOptimizationProblem mopt_problem;
  constructManifoldOptimizationValues(equalityConstrainedProblem, mopt_problem);
  constructManifoldOptimizationGraph(equalityConstrainedProblem, mopt_problem);
  return mopt_problem;
}

/* ************************************************************************* */
void ManifoldOptimizer::constructManifoldOptimizationValues(
    const EConsOptProblem& equalityConstrainedProblem,
    ManifoldOptimizationProblem& mopt_problem) const {
  constructManifoldValues(equalityConstrainedProblem, mopt_problem);
  constructUnconstrainedValues(equalityConstrainedProblem, mopt_problem);
}

/* ************************************************************************* */
void ManifoldOptimizer::constructManifoldValues(
    const EConsOptProblem& equalityConstrainedProblem,
    ManifoldOptimizationProblem& mopt_problem) const {
  IEOptimizer::ValueInitializer initialize;
  if (p_.retractInitialValues) {
    const auto creator = std::dynamic_pointer_cast<RetractorCreator>(
        p_.constraintManifoldParams->retractorCreator);
    if (!creator) {
      throw std::invalid_argument(
          "Initial equality projection requires an equality retractor policy");
    }
    initialize = [creator](const NonlinearEqualityConstraints& constraints,
                           const Values& values) {
      auto owned = std::make_shared<NonlinearEqualityConstraints>(constraints);
      return creator->create(owned)->retractConstraints(values);
    };
  }
  // Project before constructing any numerical equality basis.
  const auto discovered = IEOptimizer::identifyConnectedComponents(
      equalityConstrainedProblem.e_constraints_, NonlinearInequalityConstraints(),
      equalityConstrainedProblem.values_, p_.constraintManifoldParams,
      [](const NonlinearEqualityConstraints& equalities,
         const NonlinearInequalityConstraints&, const Values& prototype,
         const IEConstraintManifold::Params::shared_ptr& params) {
        return std::make_shared<EConstraintManifold>(equalities, prototype,
                                                     *params);
      }, initialize);
  for (const auto& type : discovered.types) {
    mopt_problem.types.push_back(
        std::dynamic_pointer_cast<const EConstraintManifold>(type));
  }
  for (const auto& entry : discovered.points) {
    auto point = ECMPoint::fromIEPoint(entry.second);
    if (point.dim() > 0) {
      mopt_problem.values.insert(entry.first, point);
      mopt_problem.manifoldKeys.insert(entry.first);
    } else {
      mopt_problem.fixedPoints.emplace(entry.first, std::move(point));
    }
  }
}

/* ************************************************************************* */
void ManifoldOptimizer::constructUnconstrainedValues(
    const EConsOptProblem& equalityConstrainedProblem,
    ManifoldOptimizationProblem& mopt_problem) {
  auto& keys = mopt_problem.unconstrainedKeys;
  keys = equalityConstrainedProblem.costs_.keys();
  for (Key key : equalityConstrainedProblem.e_constraints_.keys()) {
    keys.erase(key);
  }
  mopt_problem.values.insert(
      SubValues(equalityConstrainedProblem.values_, keys));
}

/* ************************************************************************* */
void ManifoldOptimizer::constructManifoldOptimizationGraph(
    const EConsOptProblem& equalityConstrainedProblem,
    ManifoldOptimizationProblem& mopt_problem) {
  ECMPoints allPoints = mopt_problem.points();
  allPoints.insert(mopt_problem.fixedPoints.begin(), mopt_problem.fixedPoints.end());
  mopt_problem.graph = manifoldGraph(equalityConstrainedProblem.costs_, allPoints);
}

/* ************************************************************************* */
Values ManifoldOptimizer::baseValues(const ManifoldOptimizationProblem& mopt_problem,
                                     const Values& nopt_values) const {
  Values base_values = ambientValues(mopt_problem.fixedPoints);
  for (const Key& key : mopt_problem.unconstrainedKeys) {
    base_values.insert(key, nopt_values.at(key));
  }
  for (const Key& key : mopt_problem.manifoldKeys) {
    const auto& point = nopt_values.at<ECMPoint>(key);
    if (p_.retractFinalValues) {
      // Final projection retains the previous default penalty-LM convention.
      gtsam::LevenbergMarquardtOptimizer optimizer(
          point.eConstraints()->penaltyGraph(), point.values());
      base_values.insert(optimizer.optimize());
    } else {
      base_values.insert(point.values());
    }
  }
  return base_values;
}

/* ************************************************************************* */
VectorValues ManifoldOptimizer::baseTangentVector(
    const ManifoldOptimizationProblem& mopt_problem, const Values& values,
    const VectorValues& delta) const {
  VectorValues tangentVector;
  for (const Key& key : mopt_problem.unconstrainedKeys) {
    tangentVector.insert(key, delta.at(key));
  }
  for (const Key& key : mopt_problem.manifoldKeys) {
    tangentVector.insert(values.at<ECMPoint>(key).toAmbient(delta.at(key)));
  }
  for (const auto& entry : mopt_problem.fixedPoints) {
    tangentVector.insert(entry.second.values().zeroVectors());
  }
  return tangentVector;
}

/* ************************************************************************* */
ManifoldOptimizationProblem ManifoldOptimizer::initializeManifoldOptimizationProblem(
    const gtsam::NonlinearFactorGraph& costs,
    const NonlinearEqualityConstraints& constraints,
    const gtsam::Values& init_values) const {
  EConsOptProblem equalityConstrainedProblem(costs, constraints, init_values);
  return transformProblem(equalityConstrainedProblem);
}

/* ************************************************************************* */
ManifoldOptimizationProblem ManifoldOptimizer::initializeManifoldOptimizationProblem(
    const NonlinearFactorGraph& costs, const ECMPoints& points,
    const Values& unconstrainedValues) const {
  IEOptimizer::validatePoints(widenPoints(points), unconstrainedValues);
  ManifoldOptimizationProblem problem;
  problem.values = unconstrainedValues;
  problem.unconstrainedKeys = unconstrainedValues.keySet();
  std::set<const EConstraintManifold*> seen;
  for (const auto& entry : points) {
    const auto type = entry.second.manifold();
    if (seen.insert(type.get()).second) problem.types.push_back(type);
    if (entry.second.dim() == 0) {
      problem.fixedPoints.emplace(entry.first, entry.second);
    } else {
      problem.values.insert(entry.first, entry.second);
      problem.manifoldKeys.insert(entry.first);
    }
  }
  problem.graph = manifoldGraph(costs, points);
  return problem;
}

}  // namespace gtdynamics

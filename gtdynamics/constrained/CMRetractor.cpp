/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
 * -------------------------------------------------------------------------- */
/**
 * @file CMRetractor.cpp
 * @brief Shared metric binding and mutable-workspace isolation.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */
#include <gtdynamics/constrained/CMPoint.h>
#include <gtdynamics/constrained/CMRetractor.h>
#include <gtdynamics/factors/ConstVarFactor.h>

namespace gtdynamics {
using namespace gtsam;

void CMRetractorParams::snapshotMetric(const CMRetractorParams& source,
                                       const KeySet* scope) {
  // Capture the source first so in-place scoping also retains its entries.
  const auto sourceMetric = source.metricSigmas;
  metricSigmas.reset();
  if (!sourceMetric) return;
  if (!scope) {
    metricSigmas = std::make_shared<VectorValues>(*sourceMetric);
  } else {
    metricSigmas = std::make_shared<VectorValues>();
    for (Key key : *scope)
      if (sourceMetric->exists(key))
        metricSigmas->insert(key, sourceMetric->at(key));
  }
}

CMRetractorParams::shared_ptr CMRetractorParams::snapshot(
    const shared_ptr& source, const KeySet* scope) {
  if (!source) throw std::invalid_argument("Missing retraction settings");
  auto result = std::make_shared<CMRetractorParams>(*source);
  result->snapshotMetric(*source, scope);
  return result;
}

VectorValues CMRetractorParams::priorPrecisions(const Values& values,
                                                const KeyVector& keys) const {
  if (useVaryingSigma && !metricSigmas)
    throw std::invalid_argument("Missing projection metric sigmas");
  double scale = 1.0;
  if (useVaryingSigma && scaleVaryingSigma) {
    // Match AddGeneralPriors' nominal-sigma convention.
    double minimum = 1e10;
    for (Key key : keys)
      if (metricSigmas->exists(key)) {
        const auto& sigma = metricSigmas->at(key);
        if (!sigma.size() || sigma.size() != values.at(key).dim() ||
            !sigma.allFinite() || (sigma.array() <= 0).any())
          throw std::invalid_argument("Invalid projection metric sigmas");
        minimum = std::min(minimum, sigma.minCoeff());
      }
    scale = priorSigma / minimum;
  }
  VectorValues precisions;
  for (Key key : keys) {
    if (useVaryingSigma && !metricSigmas->exists(key)) continue;
    const size_t dim = values.at(key).dim();
    const Vector sigma = useVaryingSigma ? Vector(scale * metricSigmas->at(key))
                                         : Vector::Constant(dim, priorSigma);
    if (sigma.size() != dim || !sigma.allFinite() || (sigma.array() <= 0).any())
      throw std::invalid_argument("Invalid projection metric sigmas");
    const Vector weights = sigma.array().square().inverse();
    if (!weights.allFinite() || (weights.array() <= 0).any())
      throw std::invalid_argument("Invalid projection metric precision");
    // Repeated prior keys have the same semantics as repeated prior factors.
    if (precisions.exists(key))
      precisions.at(key) += weights;
    else
      precisions.insert(key, weights);
  }
  return precisions;
}

CMRetractor::shared_ptr CMRetractor::bindPoint(const CMPoint& point) const {
  auto result = cloneWorkspace();
  if (commonParams().metricSigmas) {
    VectorValues bound;
    for (const auto& [slot, key] : point.bindings())
      if (commonParams().metricSigmas->exists(slot))
        bound.insert(key, commonParams().metricSigmas->at(slot));
    result->setMetricSigmas(bound);
  }
  return result;
}

void CMRetractor::scopeMetric(const KeySet& keys) {
  auto& params = mutableCommonParams();
  params.snapshotMetric(params, &keys);
}

void CMRetractor::setMetricSigmas(const VectorValues& sigmas) {
  mutableCommonParams().metricSigmas = std::make_shared<VectorValues>(sigmas);
}

void CMRetractor::detachFixedFactors(
    NonlinearFactorGraph& graph,
    std::vector<std::shared_ptr<ConstVarFactor>>& factors) {
  std::map<const ConstVarFactor*, std::shared_ptr<ConstVarFactor>> copies;
  for (auto& factor : factors) {
    auto copy = std::make_shared<ConstVarFactor>(*factor);
    copies.emplace(factor.get(), copy);
    factor = copy;
  }
  for (auto& factor : graph) {
    const auto* fixed = dynamic_cast<const ConstVarFactor*>(factor.get());
    if (fixed && copies.count(fixed)) factor = copies.at(fixed);
  }
}
}  // namespace gtdynamics

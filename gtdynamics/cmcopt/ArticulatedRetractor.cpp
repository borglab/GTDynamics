/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
#include <gtdynamics/cmcopt/ArticulatedRetractor.h>
#include <gtdynamics/cmcopt/CMPoint.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/linear/JacobianFactor.h>

#include <cmath>
#include <set>

namespace gtdynamics {
namespace {
// Robot topology is independent of numerical values and ambient bindings.
LinkSharedPtr validatedRoot(const std::shared_ptr<const Robot>& robot,
                            const std::string& rootName) {
  if (!robot) throw std::invalid_argument("Missing articulated robot");
  const auto root = robot->link(rootName);
  if (!root || root->isFixed())
    throw std::invalid_argument(
        "ArticulatedRetractor requires a floating root");
  for (const auto& link : robot->links())
    if (link->isFixed())
      throw std::invalid_argument("Fixed links are unsupported");
  std::set<int> reached{root->id()};
  for (size_t pass = 0; pass < robot->links().size(); ++pass)
    for (const auto& joint : robot->joints())
      if (reached.count(joint->parent()->id()))
        reached.insert(joint->child()->id());
  std::set<int> children;
  for (const auto& joint : robot->joints())
    if (!children.insert(joint->child()->id()).second ||
        joint->child() == root)
      throw std::invalid_argument("ArticulatedRetractor requires a tree");
  if (reached.size() != robot->links().size() ||
      robot->joints().size() + 1 != robot->links().size())
    throw std::invalid_argument(
        "ArticulatedRetractor requires one connected tree");
  return root;
}
}  // namespace

ArticulatedRetractor::ArticulatedRetractor(
    const CMPoint& point,
    const std::shared_ptr<const Robot>& robot, const std::string& rootName,
    const KeySet& independentAD, const IERetractorParams::shared_ptr& params,
    const ReducedALRetractor::Settings& settings)
    : ArticulatedRetractor(point, robot, validatedRoot(robot, rootName),
                           independentAD, params, settings) {}

ArticulatedRetractor::ArticulatedRetractor(
    const CMPoint& point, const std::shared_ptr<const Robot>& robot,
    const LinkSharedPtr& root, const KeySet& independentAD,
    const IERetractorParams::shared_ptr& params,
    const ReducedALRetractor::Settings& settings)
    : IERetractor(params ? std::make_shared<IERetractorParams>(*params)
                         : nullptr),
      robot_(robot),
      root_(root),
      settings_(settings) {
  if (!robot_ || !params_ || point.values().empty())
    throw std::invalid_argument(
        "ArticulatedRetractor requires robot, parameters and values");
  if (!(params_->feasibleThreshold > 0) ||
      !std::isfinite(params_->feasibleThreshold))
    throw std::invalid_argument("Invalid articulated feasibility tolerance");
  slice_ = DynamicsSymbol(point.values().keys().front()).time();
  for (const auto& link : robot_->links()) {
    qvKeys_.insert(PoseKey(link->id(), slice_));
    qvKeys_.insert(TwistKey(link->id(), slice_));
  }
  for (const auto& joint : robot_->joints()) {
    qvKeys_.insert(JointAngleKey(joint->id(), slice_));
    qvKeys_.insert(JointVelKey(joint->id(), slice_));
  }
  for (Key key : qvKeys_)
    if (!point.values().exists(key))
      throw std::invalid_argument("Missing articulated q/v variable");
  const Key worldVelocity = LinearVelocityKey(root_->id(), slice_);
  if (point.values().exists(worldVelocity)) qvKeys_.insert(worldVelocity);
  for (Key key : point.values().keys()) {
    if (DynamicsSymbol(key).time() != slice_)
      throw std::invalid_argument(
          "ArticulatedRetractor requires one time slice");
    if (independentAD.exists(key)) {
      if (qvKeys_.exists(key))
        throw std::invalid_argument(
            "q/v key in independent mechanics coordinates");
      independentAD_.insert(key);
      priorKeys_.push_back(key);
    }
  }
  if (params_->metricSigmas)
    params_->metricSigmas =
        std::make_shared<VectorValues>(*params_->metricSigmas);
  for (const auto& constraint : *point.iConstraints())
    for (Key key : constraint->keys())
      if (qvKeys_.exists(key))
        throw std::invalid_argument(
            "ArticulatedRetractor does not support q/v inequalities");
  NonlinearFactorGraph ad;
  for (const auto& factor : point.eConstraints()->penaltyGraph()) {
    bool hasAD = false;
    for (Key key : factor->keys()) hasAD |= !qvKeys_.exists(key);
    (hasAD ? ad : qvGraph_).push_back(factor);
  }
  if (ad.empty())
    throw std::invalid_argument("Missing articulated mechanics constraints");
  std::tie(adGraph_, fixedFactors_) = ConstVarGraph(ad, qvKeys_);
  const auto mechanicsKeys = adGraph_.keys();
  for (const auto& constraint : *point.iConstraints())
    for (Key key : constraint->keys())
      if (!mechanicsKeys.exists(key))
        throw std::invalid_argument(
            "Inequality key is outside articulated mechanics");
}

CMPoint ArticulatedRetractor::retract(
    const CMPoint* point, const VectorValues& delta,
    const std::optional<IndexSet>& blocking, IERetractionInfo* info) const {
  if (!point) throw std::invalid_argument("Null articulated manifold");
  Values trial = point->values().retract(delta);
  Values known;
  const Key rootPoseKey = PoseKey(root_->id(), slice_);
  const Key rootTwistKey = TwistKey(root_->id(), slice_);
  known.insert(rootPoseKey, trial.at<Pose3>(rootPoseKey));
  known.insert(rootTwistKey, trial.at(rootTwistKey));
  for (const auto& joint : robot_->joints()) {
    for (Key key :
         {JointAngleKey(joint->id(), slice_), JointVelKey(joint->id(), slice_)})
      known.insert(key, trial.at<double>(key));
  }
  const Values fk = robot_->forwardKinematics(known, slice_, root_->name());
  for (Key key : fk.keys()) {
    if (!trial.exists(key)) continue;
    if (DynamicsSymbol(key).label() == "V" &&
        dynamic_cast<const GenericValue<Vector>*>(&trial.at(key)))
      trial.update(key, Vector(fk.at<Vector6>(key)));
    else
      trial.update(key, fk.at(key));
  }
  const Vector3 worldVelocity = trial.at<Pose3>(rootPoseKey).rotation() *
                                trial.at<Vector6>(rootTwistKey).tail<3>();
  const Key worldKey = LinearVelocityKey(root_->id(), slice_);
  if (trial.exists(worldKey)) {
    if (dynamic_cast<const GenericValue<Vector>*>(&trial.at(worldKey)))
      trial.update(worldKey, Vector(worldVelocity));
    else
      trial.update(worldKey, worldVelocity);
  }
  const double qvError = qvGraph_.error(trial);
  if (!std::isfinite(qvError) ||
      qvError > params_->feasibleThreshold * params_->feasibleThreshold)
    throw std::invalid_argument(
        "ArticulatedRetractor cannot recover the supplied q/v constraints by "
        "forward kinematics");
  for (const auto& factor : fixedFactors_) factor->setFixedValues(trial);
  const auto equalities = std::make_shared<NonlinearEqualityConstraints>(
      NonlinearEqualityConstraints::FromCostGraph(adGraph_));
  const Values adTrial = SubValues(trial, adGraph_.keys());
  NonlinearFactorGraph priors;
  params_->addPriors(adTrial, priorKeys_, priors);
  auto linear = priors.linearize(adTrial);
  const auto adLinear = adGraph_.linearize(adTrial);
  for (const auto& factor : *adLinear) {
    const JacobianFactor jacobian(*factor);
    std::vector<std::pair<Key, Matrix>> terms;
    for (auto it = jacobian.begin(); it != jacobian.end(); ++it)
      terms.emplace_back(*it, jacobian.getA(it));
    linear->emplace_shared<JacobianFactor>(
        terms, jacobian.getb(), noiseModel::Constrained::All(jacobian.rows()));
  }
  const Values anchor = adTrial.retract(linear->optimize(EliminateQR));
  const double anchorViolation = equalities->violationNorm(anchor);
  if (!std::isfinite(anchorViolation) ||
      anchorViolation > params_->feasibleThreshold)
    throw std::invalid_argument(
        "ArticulatedRetractor requires mechanics affine in "
        "acceleration/wrench/torque variables");
  const auto localParams = std::make_shared<IEConstraintManifold::Params>();
  localParams->affineEqualities = true;
  const auto independent = independentAD_;
  localParams->equalityBasisCreator = std::make_shared<EliminationBasisCreator>(
      [independent](const KeyVector& keys) {
        KeyVector selected;
        for (Key key : keys)
          if (independent.exists(key)) selected.push_back(key);
        return selected;
      });
  localParams->retractorCreator = std::make_shared<UniversalIERetractorCreator>(
      std::make_shared<ReducedALRetractor>(params_, settings_, priorKeys_));
  const auto adType = std::make_shared<IEConstraintManifold>(
      *equalities, *point->iConstraints(), anchor, *localParams);
  CMPoint adPoint = adType->point(anchor);
  const auto projected =
      adPoint.retract(anchor.zeroVectors(), blocking, info);
  trial.update(projected.values());
  const double violation = point->eConstraints()->violationNorm(trial);
  if (violation > params_->feasibleThreshold || !std::isfinite(violation))
    throw RetractionFailure("Articulated retraction equality violation: " +
                            std::to_string(violation));
  return point->manifold()->point(point->bindings(), trial, projected.activeIndices());
}

ArticulatedRetractorCreator::ArticulatedRetractorCreator(
    const std::shared_ptr<const Robot>& robot, const std::string& rootName,
    const KeySet& independentAD, const IERetractorParams::shared_ptr& params,
    const ReducedALRetractor::Settings& settings)
    : IERetractorCreator(params ? std::make_shared<IERetractorParams>(*params)
                                : nullptr),
      robot_(robot),
      rootName_(rootName),
      independentAD_(independentAD),
      projectionParams_(params),
      settings_(settings) {
  if (!robot_ || !params_)
    throw std::invalid_argument("Missing articulated retractor inputs");
  root_ = validatedRoot(robot_, rootName_);
  // The outer optimizer must not replace a caller-specified projection metric.
  params_->useVaryingSigma = false;
  params_->metricSigmas.reset();
}
IERetractorCreator::shared_ptr ArticulatedRetractorCreator::cloneFor(
    const KeySet& keys) const {
  KeySet independent;
  for (Key key : keys)
    if (independentAD_.exists(key)) independent.insert(key);
  auto projection = IERetractorParams::snapshot(projectionParams_, &keys);
  auto result = std::make_shared<ArticulatedRetractorCreator>(*this);
  result->independentAD_ = std::move(independent);
  result->projectionParams_ = std::move(projection);
  return result;
}
IERetractor::shared_ptr ArticulatedRetractorCreator::create(
    const CMPoint& point) const {
  KeySet independent;
  for (Key slot : independentAD_) {
    const auto binding = point.bindings().find(slot);
    if (binding != point.bindings().end())
      independent.insert(binding->second);
  }
  auto projection = IERetractorParams::snapshot(projectionParams_);
  if (projection->metricSigmas) {
    auto bound = std::make_shared<VectorValues>();
    for (const auto& [slot, key] : point.bindings())
      if (projection->metricSigmas->exists(slot))
        bound->insert(key, projection->metricSigmas->at(slot));
    projection->metricSigmas = bound;
  }
  return IERetractor::shared_ptr(new ArticulatedRetractor(
      point, robot_, root_, independent, projection, settings_));
}
}  // namespace gtdynamics

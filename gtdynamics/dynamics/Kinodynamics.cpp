/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file Kinodynamics.cpp
 * @brief Couple articulated mechanics across time with trapezoidal collocation.
 * @author GTDynamics contributors
 */
#include <gtdynamics/cmcopt/IELMOptimizer.h>
#include <gtdynamics/cmcopt/IndependentComponents.h>
#include <gtdynamics/cmcopt/ReducedALRetractor.h>
#include <gtdynamics/dynamics/ArticulatedDynamics.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/dynamics/Kinodynamics.h>
#include <gtdynamics/dynamics/WorldVelocityFactor.h>
#include <gtdynamics/factors/CollocationFactors.h>
#include <gtdynamics/factors/ConstVarFactor.h>
#include <gtdynamics/kinematics/ContactKinematicsTwistFactor.h>
#include <gtdynamics/kinematics/TwistFactor.h>
#include <gtdynamics/utils/PoseContactInitializer.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>

#include <chrono>
#include <iostream>

namespace gtdynamics {
namespace {
using namespace gtsam;
using Clock = std::chrono::steady_clock;
double elapsed(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

// The q/v recovery is exact articulated FK. Once q/v are recovered, all
// remaining mechanics are affine in accelerations/wrenches/torques. Their
// equality anchor is a constrained Gaussian QR solve; the original sole cones
// are imposed by the existing reduced AL retractor. The enclosing full-state
// manifold remains nonlinear and relinearizes at every new configuration.
class ArticulatedRetractor : public IERetractor {
  const Robot& robot_;
  LinkSharedPtr root_;
  size_t slice_;
  KeySet qvKeys_;
  NonlinearFactorGraph adGraph_;
  std::vector<std::shared_ptr<ConstVarFactor>> fixedFactors_;
  KeySet independentAD_;
  KeyVector priorKeys_;

 public:
  ArticulatedRetractor(const IEConstraintManifold& manifold, const Robot& robot,
                       const LinkSharedPtr& root, const KeySet& independentAD,
                       const IERetractorParams::shared_ptr& params)
      : IERetractor(std::make_shared<IERetractorParams>(*params)),
        robot_(robot),
        root_(root),
        slice_(DynamicsSymbol(manifold.values().keys().front()).time()),
        independentAD_() {
    for (Key key : manifold.values().keys())
      if (independentAD.exists(key)) independentAD_.insert(key);
    for (const auto& link : robot.links()) {
      qvKeys_.insert(PoseKey(link->id(), slice_));
      qvKeys_.insert(TwistKey(link->id(), slice_));
    }
    for (const auto& joint : robot.joints()) {
      qvKeys_.insert(JointAngleKey(joint->id(), slice_));
      qvKeys_.insert(JointVelKey(joint->id(), slice_));
    }
    qvKeys_.insert(LinearVelocityKey(root->id(), slice_));
    // Project in independent acceleration/wrench coordinates. Priors on every
    // dependent reaction wrench would distort the metric and create a poorly
    // conditioned projection of the same small affine AD manifold.
    params_->useVaryingSigma = true;
    params_->metricSigmas = std::make_shared<VectorValues>();
    const double rootAccelerationSigmaDegrees = 90.0;
    const double jointAccelerationSigmaDegrees = 1000.0;
    for (const auto& [key, dimension] : manifold.values().dims()) {
      if (!independentAD_.exists(key)) continue;
      priorKeys_.push_back(key);
      Vector sigmas;
      if (key == TwistAccelKey(root_->id(), slice_)) {
        sigmas = Vector6::Constant(rootAccelerationSigmaDegrees *
                                   std::acos(-1.0) / 180.0);
        sigmas.tail<3>().setConstant(3.0);
      } else if (DynamicsSymbol(key).label() == "a") {
        sigmas =
            Vector1(jointAccelerationSigmaDegrees * std::acos(-1.0) / 180.0);
      } else {
        sigmas = Vector6::Constant(500.0);
        sigmas.head<3>().setConstant(30.0);
      }
      params_->metricSigmas->insert(key, sigmas);
    }
    NonlinearFactorGraph ad;
    for (const auto& factor : manifold.eConstraints()->penaltyGraph()) {
      bool hasAD = false;
      for (Key key : factor->keys()) hasAD |= !qvKeys_.exists(key);
      if (hasAD) ad.push_back(factor);
    }
    std::tie(adGraph_, fixedFactors_) = ConstVarGraph(ad, qvKeys_);
  }

  IEConstraintManifold retract(
      const IEConstraintManifold* manifold, const VectorValues& delta,
      const std::optional<IndexSet>& blocking = {},
      IERetractionInfo* info = nullptr) const override {
    Values trial = manifold->values().retract(delta);
    Values known;
    const Key rootPoseKey = PoseKey(root_->id(), slice_);
    const Key rootTwistKey = TwistKey(root_->id(), slice_);
    known.insert(rootPoseKey, trial.at<Pose3>(rootPoseKey));
    known.insert(rootTwistKey, trial.at<Vector6>(rootTwistKey));
    for (const auto& joint : robot_.joints()) {
      for (Key key : {JointAngleKey(joint->id(), slice_),
                      JointVelKey(joint->id(), slice_)})
        known.insert(key, trial.at<double>(key));
    }
    const Values fk = robot_.forwardKinematics(known, slice_, root_->name());
    for (Key key : fk.keys())
      if (trial.exists(key)) trial.update(key, fk.at(key));
    const Vector3 worldVelocity = trial.at<Pose3>(rootPoseKey).rotation() *
                                  trial.at<Vector6>(rootTwistKey).tail<3>();
    trial.update(LinearVelocityKey(root_->id(), slice_), Vector(worldVelocity));
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
          terms, jacobian.getb(),
          noiseModel::Constrained::All(jacobian.rows()));
    }
    const Values anchor = adTrial.retract(linear->optimize(EliminateQR));
    const auto localParams = std::make_shared<IEConstraintManifold::Params>();
    localParams->affineEqualities = true;
    const auto independent = independentAD_;
    localParams->equalityBasisCreator =
        std::make_shared<EliminationBasisCreator>(
            [independent](const KeyVector& keys) {
              KeyVector selected;
              for (Key key : keys)
                if (independent.exists(key)) selected.push_back(key);
              return selected;
            });
    ReducedALRetractor::Settings projectionSettings;
    // Projection stationarity is in whitened independent AD coordinates. The
    // original force/cone feasibility tolerance is 1e-7; a 1e-4 dual
    // stationarity target avoids requesting roundoff-level AL inner decreases.
    projectionSettings.al.absoluteStationarityTolerance = 1e-4;
    projectionSettings.feasibilityTolerance = 1e-7;
    projectionSettings.activeTolerance = 1e-6;
    localParams->retractorCreator =
        std::make_shared<UniversalIERetractorCreator>(
            std::make_shared<ReducedALRetractor>(params_, projectionSettings,
                                                 priorKeys_));
    IEConstraintManifold adManifold(localParams, equalities,
                                    manifold->iConstraints(), anchor);
    const auto projected =
        adManifold.retract(anchor.zeroVectors(), blocking, info);
    trial.update(projected.values());
    const double violation = manifold->eConstraints()->violationNorm(trial);
    if (violation > 1e-6 || !std::isfinite(violation))
      throw std::runtime_error("Articulated retraction equality violation: " +
                               std::to_string(violation));
    return manifold->createWithNewValues(trial, projected.activeIndices());
  }
};

class ArticulatedRetractorCreator : public IERetractorCreator {
  const Robot& robot_;
  LinkSharedPtr root_;
  KeySet independentAD_;

 public:
  ArticulatedRetractorCreator(const Robot& robot, const LinkSharedPtr& root,
                              const KeySet& independentAD)
      : robot_(robot), root_(root), independentAD_(independentAD) {
    params_->ensureFeasible = true;
    params_->feasibleThreshold = 1e-7;
  }
  IERetractor::shared_ptr create(
      const IEConstraintManifold& manifold) const override {
    return std::make_shared<ArticulatedRetractor>(manifold, robot_, root_,
                                                  independentAD_, params_);
  }
};
// Retire each stance landmark at its final observation, keeping a shared bias
// until its last factor. Time-block elimination preserves the chain structure.
Ordering chronologicalOrdering(const NonlinearFactorGraph& cost,
                               const IEManifoldValues& manifolds,
                               const Values& unconstrained, size_t firstSlice) {
  const auto ownership = IEOptimizer::varToManifoldKeyMap(manifolds);
  std::map<Key, size_t> lastSlice;
  for (Key key : unconstrained.keys()) lastSlice[key] = firstSlice;
  for (const auto& factor : cost) {
    size_t last = firstSlice;
    for (Key key : factor->keys()) {
      const auto owner = ownership.find(key);
      if (owner != ownership.end())
        last = std::max(last, DynamicsSymbol(owner->second).time());
    }
    for (Key key : factor->keys())
      if (unconstrained.exists(key))
        lastSlice[key] = std::max(lastSlice[key], last);
  }
  std::map<size_t, Key> chronological;
  for (const auto& [key, manifold] : manifolds)
    chronological.emplace(DynamicsSymbol(key).time(), key);
  Ordering ordering;
  for (const auto& [k, key] : chronological) {
    ordering.push_back(key);
    for (const auto& [freeKey, last] : lastSlice)
      if (last == k) ordering.push_back(freeKey);
  }
  return ordering;
}
}  // namespace

kinodynamics::Result solveKinodynamics(
    const Robot& robot, const Trajectory& trajectory,
    const std::vector<Slice>& slices, const std::vector<double>& times,
    const gtsam::Values& initialValues,
    const std::vector<gtsam::NonlinearFactorGraph>& measurementGroups,
    const std::vector<std::string>& measurementNames,
    const std::vector<fixed_dynamics::Contact>& contacts,
    const gtsam::Vector& armature, const kinodynamics::Options& options) {
  using namespace gtsam;
  const auto assemblyStart = Clock::now();
  if (measurementGroups.size() != measurementNames.size() ||
      slices.size() < 2 || !options.maxIterations ||
      options.retractionThreads < 1)
    throw std::invalid_argument(
        "Kinodynamics requires named groups and >=2 slices");
  for (double sigma :
       {options.poseRotationSigmaDegrees, options.poseTranslationSigma,
        options.twistAngularSigmaDegrees, options.twistLinearSigma,
        options.jointPositionSigmaDegrees, options.jointVelocitySigmaDegrees,
        options.stanceVelocitySigma})
    if (!(sigma > 0) || !std::isfinite(sigma))
      throw std::invalid_argument(
          "Kinodynamics sigmas must be finite and positive");
  if (std::none_of(slices.begin(), slices.end(), [&](const Slice& slice) {
        return slice.k == options.gaugeSlice;
      }))
    throw std::invalid_argument("The gauge slice must be in the trajectory");
  LinkSharedPtr root;
  std::set<int> children;
  for (const auto& joint : robot.joints())
    children.insert(joint->child()->id());
  for (const auto& link : robot.links())
    if (!children.count(link->id())) root = link;
  if (!root) throw std::invalid_argument("Missing floating root");
  Values qv, accelerationPriors;
  for (const Slice& slice : slices) {
    for (const auto& link : robot.links()) {
      qv.insert(PoseKey(link->id(), slice.k),
                initialValues.at<Pose3>(PoseKey(link->id(), slice.k)));
      qv.insert(TwistKey(link->id(), slice.k), Vector(Vector6::Zero()));
    }
    accelerationPriors.insert(TwistAccelKey(root->id(), slice.k),
                              Vector(Vector6::Zero()));
    for (const auto& joint : robot.joints()) {
      qv.insert(JointAngleKey(joint->id(), slice.k),
                initialValues.at<double>(JointAngleKey(joint->id(), slice.k)));
      qv.insert(JointVelKey(joint->id(), slice.k), 0.0);
      accelerationPriors.insert(JointAccelKey(joint->id(), slice.k), 0.0);
    }
  }
  fixed_dynamics::Options mechanics;
  mechanics.baseAngularAccelerationSigmaDegrees = 1000;
  mechanics.baseLinearAccelerationSigma = 30;
  mechanics.jointAccelerationSigmaDegrees = 2000;
  internal::ArticulatedDynamicsProblem problem(robot, trajectory, slices, times,
                                               qv, accelerationPriors, contacts,
                                               armature, mechanics, false);
  NonlinearFactorGraph exact = problem.equalities.penaltyGraph();
  auto groups = measurementGroups;
  auto names = measurementNames;
  NonlinearFactorGraph poseCollocation, velocityCollocation, stanceVelocity;
  const double radiansPerDegree = std::acos(-1.0) / 180.0;
  Vector6 poseSigmas, twistSigmas;
  poseSigmas << Vector3::Constant(options.poseRotationSigmaDegrees *
                                  radiansPerDegree),
      Vector3::Constant(options.poseTranslationSigma);
  twistSigmas << Vector3::Constant(options.twistAngularSigmaDegrees *
                                   radiansPerDegree),
      Vector3::Constant(options.twistLinearSigma);
  const auto poseNoise = noiseModel::Diagonal::Sigmas(poseSigmas);
  const auto twistNoise = noiseModel::Diagonal::Sigmas(twistSigmas);
  const auto qNoise = noiseModel::Isotropic::Sigma(
      1, options.jointPositionSigmaDegrees * radiansPerDegree);
  const auto vNoise = noiseModel::Isotropic::Sigma(
      1, options.jointVelocitySigmaDegrees * radiansPerDegree);
  KeySet independent = problem.independentKeys;
  const auto tracked = contactPoseFrames(trajectory);
  for (size_t i = 0; i < slices.size(); ++i) {
    const size_t k = slices[i].k;
    const Key poseKey = PoseKey(root->id(), k),
              twistKey = TwistKey(root->id(), k);
    problem.zero.insert(LinearVelocityKey(root->id(), k),
                        Vector(Vector3::Zero()));
    exact.emplace_shared<WorldVelocityFactor>(poseKey, twistKey,
                                              LinearVelocityKey(root->id(), k),
                                              noiseModel::Unit::Create(3));
    independent.insert(twistKey);
    if (k != options.gaugeSlice)
      independent.insert(poseKey);
    else
      exact.emplace_shared<PriorFactor<Pose3>>(poseKey, qv.at<Pose3>(poseKey),
                                               noiseModel::Unit::Create(6));
    for (const auto& joint : robot.joints()) {
      const int id = joint->id();
      independent.insert(JointAngleKey(id, k));
      independent.insert(JointVelKey(id, k));
      exact.emplace_shared<ExpressionFactor<Vector6>>(
          noiseModel::Unit::Create(6), Vector6::Zero(),
          joint->poseConstraint(k));
      exact.add(TwistFactor(noiseModel::Unit::Create(6), joint, k));
      if (i) {
        const size_t before = slices[i - 1].k;
        const double dt = times[i] - times[i - 1];
        DynamicsGraph::addCollocationFactorDouble(
            &poseCollocation, JointAngleKey(id, before), JointAngleKey(id, k),
            JointVelKey(id, before), JointVelKey(id, k), dt, qNoise);
        DynamicsGraph::addCollocationFactorDouble(
            &velocityCollocation, JointVelKey(id, before), JointVelKey(id, k),
            JointAccelKey(id, before), JointAccelKey(id, k), dt, vNoise);
      }
    }
    const auto frames = tracked.find(k);
    if (frames != tracked.end())
      for (const auto& frame : frames->second)
        stanceVelocity.push_back(ContactKinematicsTwistFactor(
            TwistKey(frame.contact.link->id(), k),
            noiseModel::Isotropic::Sigma(3, options.stanceVelocitySigma),
            frame.contact.comTcontact.inverse()));
    if (i) {
      const size_t before = slices[i - 1].k;
      const double dt = times[i] - times[i - 1];
      poseCollocation.emplace_shared<FixTimeTrapezoidalPoseCollocationFactor>(
          PoseKey(root->id(), before), poseKey, TwistKey(root->id(), before),
          twistKey, dt, poseNoise);
      velocityCollocation
          .emplace_shared<FixTimeTrapezoidalTwistCollocationFactor>(
              TwistKey(root->id(), before), twistKey,
              TwistAccelKey(root->id(), before), TwistAccelKey(root->id(), k),
              dt, twistNoise);
    }
  }
  for (Key key : initialValues.keys())
    if (!problem.zero.exists(key))
      problem.zero.insert(key, initialValues.at(key));
  groups.insert(groups.end(),
                {poseCollocation, velocityCollocation, stanceVelocity,
                 problem.effort, problem.wrench, problem.torqueSmooth,
                 problem.wrenchSmooth, problem.acceleration});
  names.insert(names.end(),
               {"Configuration collocation", "Velocity collocation",
                "Stance velocity", "Torque effort", "Contact wrench",
                "Torque rate", "Wrench rate", "Acceleration regularization"});
  NonlinearFactorGraph cost;
  for (const auto& group : groups) cost.add(group);
  problem.equalities = NonlinearEqualityConstraints::FromCostGraph(exact);
  kinodynamics::Result result;
  result.initialValues = problem.zero;
  result.initialEqualityL2 = problem.equalities.violationNorm(problem.zero);
  result.variableDimension = problem.zero.dim();
  result.equalityDimension = problem.equalities.dim();
  result.inequalityDimension = problem.inequalities.dim();
  result.errorNames = names;
  result.contactKeys = problem.result.contactKeys;
  result.torqueKeys = problem.result.torqueKeys;
  result.jointIds = problem.result.jointIds;
  result.assemblySeconds = elapsed(assemblyStart);
  auto manifoldParams = std::make_shared<IEConstraintManifold::Params>();
  manifoldParams->equalityBasisCreator =
      std::make_shared<EliminationBasisCreator>(
          [independent](const KeyVector& keys) {
            KeyVector selected;
            for (Key key : keys)
              if (independent.exists(key)) selected.push_back(key);
            return selected;
          });
  manifoldParams->retractorCreator =
      std::make_shared<ArticulatedRetractorCreator>(robot, root,
                                                    problem.independentKeys);
  auto start = Clock::now();
  auto manifolds = IEOptimizer::identifyManifolds(
      problem.equalities, problem.inequalities, problem.zero, manifoldParams);
  const auto componentKeys = manifolds.keys();
  internal::forIndependentComponents(
      componentKeys.size(), options.retractionThreads, [&](size_t i) {
        auto& manifold = manifolds.at(componentKeys[i]);
        manifold = manifold.retract(manifold.values().zeroVectors());
      });
  for (const auto& [key, manifold] : manifolds)
    result.reducedDimension += manifold.eBasis()->dim();
  Values initial = manifolds.baseValues();
  const Values unconstrained = IEOptimizer::identifyUnconstrainedValues(
      problem.equalities, problem.inequalities, problem.zero);
  initial.insert(unconstrained);
  result.initializationSeconds = elapsed(start);
  result.objectiveInitial = cost.error(initial);
  IELMParams params;
  params.historyWindow = 1;
  params.retractionThreads = options.retractionThreads;
  params.lmParams.maxIterations = options.maxIterations;
  params.lmParams.relativeErrorTol = 1e-5;
  params.lmParams.absoluteErrorTol = 1e-4;
  params.lmParams.lambdaInitial = 0.01;
  params.lmParams.setDiagonalDamping(true);
  // Eliminate one time block at a time. Multifrontal amalgamation can turn the
  // chain's dense 111-coordinate blocks plus a shared bias into one large QR
  // front when active cone faces are present. Retire each stance landmark at
  // its final observation, keeping only the shared bias until the end.
  params.lmParams.setLinearSolverType("SEQUENTIAL_CHOLESKY");
  params.lmParams.ordering =
      chronologicalOrdering(cost, manifolds, unconstrained, slices.front().k);
  params.lmParams.verbosityLM = LevenbergMarquardtParams::SUMMARY;
  const auto optimizeStart = Clock::now();
  params.iterationCallback = [&](const IELMState& state) {
    const Values values = state.baseValues();
    Vector errors(groups.size());
    for (size_t i = 0; i < groups.size(); ++i)
      errors(i) = groups[i].error(values);
    result.errorHistory.push_back(errors);
    result.wallSeconds.push_back(elapsed(optimizeStart));
    result.iterations = state.iterations;
    std::cout << "Kinodynamics iteration " << state.iterations
              << ": error=" << state.error
              << ", seconds=" << result.wallSeconds.back() << std::endl;
  };
  IELMOptimizer optimizer(params, manifoldParams);
  result.values = optimizer.optimizeManifolds(cost, manifolds, unconstrained);
  result.optimizationSeconds = elapsed(optimizeStart);
  result.objectiveFinal = cost.error(result.values);
  result.equalityL2 = problem.equalities.violationNorm(result.values);
  result.inequalityL2 = problem.inequalities.violationNorm(result.values);
  result.feasible = result.equalityL2 < 1e-5 && result.inequalityL2 < 1e-5;
  return result;
}
}  // namespace gtdynamics

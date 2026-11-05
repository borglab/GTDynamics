/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */

/**
 * @file FixedKinematicsDynamics.cpp
 * @brief Assemble and solve affine trajectory dynamics with native CMC-Opt.
 * @author GTDynamics contributors
 */

#include <gtdynamics/cmcopt/IELMOptimizer.h>
#include <gtdynamics/cmcopt/ReducedALRetractor.h>
#include <gtdynamics/dynamics/ContactWrenchFactor.h>
#include <gtdynamics/dynamics/Dynamics.h>
#include <gtdynamics/dynamics/FixedKinematicsDynamics.h>
#include <gtdynamics/dynamics/TwistAccelFactor.h>
#include <gtdynamics/dynamics/WrenchFactor.h>
#include <gtdynamics/factors/ConstVarFactor.h>
#include <gtdynamics/kinematics/PoseContactInitializer.h>
#include <gtdynamics/mechanics/Mechanics.h>
#include <gtdynamics/utils/ContactWrench.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtsam/constrained/NonlinearInequalityConstraint.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

namespace gtdynamics {
namespace {
using namespace gtsam;
using Clock = std::chrono::steady_clock;

double seconds(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

// Retain the projection backend itself, including its affine caches, while
// collecting lightweight diagnostics across initialization and all LM trials.
class CountedReducedALRetractor : public ReducedALRetractor {
 public:
  mutable size_t alSolves = 0, feasibleTrials = 0, feasibleProjections = 0;
  explicit CountedReducedALRetractor(
      const IERetractorParams::shared_ptr& params)
      : ReducedALRetractor(params) {}
  IEConstraintManifold retract(const IEConstraintManifold* manifold,
                               const VectorValues& delta,
                               const std::optional<IndexSet>& blockingIndices,
                               IERetractionInfo* info) const override {
    IERetractionInfo local;
    auto result =
        ReducedALRetractor::retract(manifold, delta, blockingIndices, &local);
    alSolves += local.numALSolveCalls;
    feasibleTrials += local.numFeasibleTrials;
    feasibleProjections += local.numFeasibleProjections;
    if (info) *info = local;
    return result;
  }
};

// Add exact *linearized* equality rows to a Gaussian objective. Kinematics is
// fixed through ConstVarGraph, so a single QR solve is an exact affine anchor,
// rather than a nonlinear initialization optimization or a dynamics substitute.
Values equalityAnchor(const NonlinearFactorGraph& cost,
                      const NonlinearEqualityConstraints& equalities,
                      const Values& zero, const Values& contactSeed) {
  auto linear = cost.linearize(zero);
  // Positive normal loads place stance wrenches inside their cones. With
  // free root/joint accelerations, affine articulated dynamics admits those
  // loads exactly. These rows choose ONLY a seed; they are absent from the
  // subsequent optimization graph. Using acceleration priors here keeps this
  // construction block diagonal across slices, without temporal force costs.
  for (Key key : contactSeed.keys())
    linear->emplace_shared<JacobianFactor>(key, Matrix6::Identity(),
                                           contactSeed.at<Vector6>(key),
                                           noiseModel::Constrained::All(6));
  const auto linearEqualities = equalities.penaltyGraph().linearize(zero);
  for (const auto& factor : *linearEqualities) {
    JacobianFactor jacobian(*factor);
    std::vector<std::pair<Key, Matrix>> terms;
    for (auto it = jacobian.begin(); it != jacobian.end(); ++it)
      terms.emplace_back(*it, jacobian.getA(it));
    linear->emplace_shared<JacobianFactor>(
        terms, jacobian.getb(), noiseModel::Constrained::All(jacobian.rows()));
  }
  return zero.retract(linear->optimize(EliminateQR));
}

struct Problem {
  Values zero, contactSeed;
  NonlinearFactorGraph cost, effort, wrench, torqueSmooth, wrenchSmooth,
      balance, acceleration;
  NonlinearFactorGraph rootBalance;
  NonlinearEqualityConstraints equalities;
  NonlinearInequalityConstraints inequalities;
  KeySet independentKeys;
  fixed_dynamics::Result result;

  Problem(const Robot& robot, const Trajectory& trajectory,
          const std::vector<Slice>& slices, const std::vector<double>& times,
          const Values& fixedValues, const Values& accelerationPriors,
          const std::vector<fixed_dynamics::Contact>& contacts,
          const Vector& armature, const fixed_dynamics::Options& options) {
    const auto start = Clock::now();
    if (slices.empty() || slices.size() != times.size() || contacts.empty())
      throw std::invalid_argument(
          "Dynamics requires matching slices/times and contacts");
    if (!options.gravity.allFinite() || options.maxIterations == 0)
      throw std::invalid_argument(
          "Invalid gravity or dynamics iteration limit");
    if (!armature.allFinite() || !options.torqueLimits.allFinite())
      throw std::invalid_argument("Armature and torque limits must be finite");
    for (double sigma : {options.effortSigma, options.wrenchSigma,
                         options.torqueSmoothSigma, options.wrenchSmoothSigma,
                         options.balanceMomentSigma, options.balanceForceSigma,
                         options.baseAngularAccelerationSigmaDegrees,
                         options.baseLinearAccelerationSigma,
                         options.jointAccelerationSigmaDegrees})
      if (!(sigma > 0) || !std::isfinite(sigma))
        throw std::invalid_argument(
            "Dynamics sigmas must be positive and finite");
    for (size_t i = 0; i < slices.size(); ++i)
      if (!std::isfinite(times[i]) ||
          (i && (times[i] <= times[i - 1] || slices[i].k <= slices[i - 1].k)))
        throw std::invalid_argument(
            "Dynamics slices and times must increase strictly");

    auto joints = robot.joints();
    std::sort(joints.begin(), joints.end(),
              [](const auto& a, const auto& b) { return a->id() < b->id(); });
    std::set<int> children;
    for (const auto& joint : joints) {
      children.insert(joint->child()->id());
      result.jointIds.push_back(joint->id());
      if ((armature.size() &&
           (joint->id() >= armature.size() || armature(joint->id()) < 0)) ||
          (options.torqueLimits.size() &&
           (joint->id() >= options.torqueLimits.size() ||
            !(options.torqueLimits(joint->id()) > 0))))
        throw std::invalid_argument(
            "Armature/torque limits must be indexed by joint id");
    }
    LinkSharedPtr root;
    for (const auto& link : robot.links())
      if (!children.count(link->id())) {
        if (root)
          throw std::invalid_argument(
              "Dynamics requires one floating tree root");
        root = link;
      }
    if (!root || root->isFixed())
      throw std::invalid_argument("Dynamics requires a floating tree root");
    double totalMass = 0;
    for (const auto& link : robot.links()) totalMass += link->mass();
    std::set<const Link*> robotLinks;
    for (const auto& link : robot.links()) robotLinks.insert(link.get());
    for (const auto& contact : contacts) {
      if (!contact.pose.link || !contact.pose.isGround)
        throw std::invalid_argument(
            "Dynamics contacts require a ground material frame");
      if (!robotLinks.count(contact.pose.link.get()))
        throw std::invalid_argument(
            "Dynamics contact link must belong to the robot");
      if (!contact.pose.comTcontact.matrix().allFinite())
        throw std::invalid_argument(
            "Dynamics contact transform must be finite");
    }

    const auto tracked = contactPoseFrames(trajectory);
    const double radiansPerDegree = std::acos(-1.0) / 180.0;
    Vector6 baseAccelerationSigmas;
    baseAccelerationSigmas << Vector3::Constant(
        options.baseAngularAccelerationSigmaDegrees * radiansPerDegree),
        Vector3::Constant(options.baseLinearAccelerationSigma);
    const auto baseAccelerationNoise =
        noiseModel::Diagonal::Sigmas(baseAccelerationSigmas);
    const auto jointAccelerationNoise = noiseModel::Isotropic::Sigma(
        1, options.jointAccelerationSigmaDegrees * radiansPerDegree);
    DynamicsParameters parameters(1.0);
    parameters.gravity = options.gravity;
    const Dynamics dynamics(parameters);
    const Mechanics mechanics(parameters);
    Values prescribed;
    NonlinearFactorGraph exact;
    for (size_t i = 0; i < slices.size(); ++i) {
      const size_t k = slices[i].k;
      std::map<int, std::vector<Key>> external;
      std::vector<size_t> active;
      const auto frames = tracked.find(k);
      for (size_t c = 0; c < contacts.size(); ++c) {
        const auto& contact = contacts[c];
        const int id = contact.pose.link->id();
        // ContactWrenchKey is the public sole wrench. The separate material
        // transform factor connects it to the wrench used by COM dynamics.
        const Key soleKey = ContactWrenchKey(id, c, k);
        const Key comKey = DynamicsSymbol::LinkJointSymbol("ec", id, c, k);
        result.contactKeys.push_back(soleKey);
        zero.insert(soleKey, Vector(Vector6::Zero()));
        zero.insert(comKey, Vector(Vector6::Zero()));
        external[id].push_back(comKey);
        exact.emplace_shared<ContactWrenchFactor>(soleKey, comKey,
                                                  contact.pose.comTcontact,
                                                  noiseModel::Unit::Create(6));
        std::optional<size_t> stance;
        if (frames != tracked.end())
          for (const auto& frame : frames->second)
            if (frame.contact.equals(contact.pose)) stance = frame.stance;
        if (stance) {
          active.push_back(c);
          const auto cone =
              RectangularContactWrench(contact.halfLength, contact.halfWidth,
                                       contact.frictionCoefficient);
          for (const auto& inequality : cone.constraints(soleKey))
            inequalities.push_back(inequality);
        } else {
          exact.emplace_shared<PriorFactor<Vector6>>(
              soleKey, Vector6::Zero(), noiseModel::Unit::Create(6));
        }
        const double length = std::sqrt(contact.halfLength * contact.halfWidth);
        Vector6 sigmas;
        sigmas << Vector3::Constant(options.wrenchSigma * length),
            Vector3::Constant(options.wrenchSigma);
        wrench.emplace_shared<PriorFactor<Vector6>>(
            soleKey, Vector6::Zero(), noiseModel::Diagonal::Sigmas(sigmas));
        bool sameStance = false;
        if (i && stance) {
          const auto previous = tracked.find(slices[i - 1].k);
          if (previous != tracked.end())
            for (const auto& frame : previous->second)
              if (frame.contact.equals(contact.pose) && frame.stance == *stance)
                sameStance = true;
        }
        if (sameStance) {
          sigmas *= options.wrenchSmoothSigma / options.wrenchSigma *
                    std::sqrt(times[i] - times[i - 1]);
          wrenchSmooth.emplace_shared<BetweenFactor<Vector6>>(
              ContactWrenchKey(id, c, slices[i - 1].k), soleKey,
              Vector6::Zero(), noiseModel::Diagonal::Sigmas(sigmas));
          ++result.wrenchSmoothFactors;
        }
      }
      if (active.empty() && !options.softDynamics)
        throw std::invalid_argument(
            "Strict fixed dynamics requires support at every slice");

      for (size_t c = 0; c < contacts.size(); ++c) {
        Vector6 load = Vector6::Zero();
        if (std::find(active.begin(), active.end(), c) != active.end())
          load(5) = totalMass * options.gravity.norm() / active.size();
        contactSeed.insert(ContactWrenchKey(contacts[c].pose.link->id(), c, k),
                           Vector(load));
      }

      // Root Newton-Euler balance determines one active six-axis contact
      // wrench. All other active contact wrenches are independent coordinates.
      // Removing that one root equality in the explicit soft mode gives every
      // active sole wrench its six coordinates without changing internal
      // physics.
      for (size_t a = options.softDynamics ? 0 : 1; a < active.size(); ++a)
        independentKeys.insert(ContactWrenchKey(
            contacts[active[a]].pose.link->id(), active[a], k));

      for (const auto& link : robot.links()) {
        for (const auto& joint : link->joints()) {
          const Key key = WrenchKey(link->id(), joint->id(), k);
          zero.insert(key, Vector(Vector6::Zero()));
        }
        // Missing values fail here with the actual DynamicsSymbol key.
        prescribed.insert(PoseKey(link->id(), k),
                          fixedValues.at<Pose3>(PoseKey(link->id(), k)));
        prescribed.insert(TwistKey(link->id(), k),
                          fixedValues.at<Vector6>(TwistKey(link->id(), k)));
        const Key accelerationKey = TwistAccelKey(link->id(), k);
        if (fixedValues.exists(accelerationKey))
          throw std::invalid_argument(
              "Acceleration means belong in accelerationPriors, not "
              "fixedValues");
        zero.insert(accelerationKey, Vector(Vector6::Zero()));
        if (link == root) {
          const Vector6 mean = accelerationPriors.at<Vector6>(accelerationKey);
          if (!mean.allFinite())
            throw std::invalid_argument(
                "Root acceleration prior must be finite");
          independentKeys.insert(accelerationKey);
          acceleration.emplace_shared<PriorFactor<Vector6>>(
              accelerationKey, mean, baseAccelerationNoise);
        }
      }
      for (const auto& factor :
           dynamics.wrenchBalanceFactors(Slice(k), robot, external)) {
        if (std::find(factor->keys().begin(), factor->keys().end(),
                      TwistAccelKey(root->id(), k)) == factor->keys().end()) {
          exact.add(factor);
        } else {
          rootBalance.add(factor);
          if (options.softDynamics) {
            Vector6 sigmas;
            sigmas << Vector3::Constant(options.balanceMomentSigma),
                Vector3::Constant(options.balanceForceSigma);
            balance.add(std::dynamic_pointer_cast<NoiseModelFactor>(factor)
                            ->cloneWithNewNoiseModel(
                                noiseModel::Diagonal::Sigmas(sigmas)));
          } else {
            exact.add(factor);
          }
        }
      }
      exact.add(dynamics.aFactors(Slice(k), robot));
      exact.add(mechanics.wrenchEquivalenceFactors(Slice(k), robot));
      exact.add(mechanics.torqueFactors(Slice(k), robot, armature));
      for (const auto& joint : joints) {
        const int id = joint->id();
        prescribed.insert(JointAngleKey(id, k),
                          fixedValues.at<double>(JointAngleKey(id, k)));
        prescribed.insert(JointVelKey(id, k),
                          fixedValues.at<double>(JointVelKey(id, k)));
        const Key accelerationKey = JointAccelKey(id, k);
        if (fixedValues.exists(accelerationKey))
          throw std::invalid_argument(
              "Acceleration means belong in accelerationPriors, not "
              "fixedValues");
        const double mean = accelerationPriors.at<double>(accelerationKey);
        if (!std::isfinite(mean))
          throw std::invalid_argument(
              "Joint acceleration prior must be finite");
        zero.insert(accelerationKey, 0.0);
        independentKeys.insert(accelerationKey);
        acceleration.emplace_shared<PriorFactor<double>>(
            accelerationKey, mean, jointAccelerationNoise);
        const Key torqueKey = TorqueKey(id, k);
        result.torqueKeys.push_back(torqueKey);
        zero.insert(torqueKey, 0.0);
        effort.emplace_shared<PriorFactor<double>>(
            torqueKey, 0.0,
            noiseModel::Isotropic::Sigma(1, options.effortSigma));
        if (i)
          torqueSmooth.emplace_shared<BetweenFactor<double>>(
              TorqueKey(id, slices[i - 1].k), torqueKey, 0.0,
              noiseModel::Isotropic::Sigma(
                  1, options.torqueSmoothSigma *
                         std::sqrt(times[i] - times[i - 1])));
        if (options.torqueLimits.size()) {
          const Double_ torque(torqueKey), limit(options.torqueLimits(id));
          inequalities.push_back(
              ScalarExpressionInequalityConstraint::LeqZero(torque - limit, 1));
          inequalities.push_back(ScalarExpressionInequalityConstraint::LeqZero(
              Double_(0.0) - torque - limit, 1));
        }
      }
    }
    // Substitute configurations and velocities only. Native acceleration
    // kinematics and Newton-Euler balance stay affine in the free
    // accelerations, reaction wrenches, sole wrenches, and motor torques.
    exact = ConstVarGraph(exact, prescribed);
    rootBalance = ConstVarGraph(rootBalance, prescribed);
    balance = ConstVarGraph(balance, prescribed);
    equalities = NonlinearEqualityConstraints::FromCostGraph(exact);
    cost.add(effort);
    cost.add(wrench);
    cost.add(torqueSmooth);
    cost.add(wrenchSmooth);
    cost.add(balance);
    cost.add(acceleration);
    result.softDynamics = options.softDynamics;
    result.variableDimension = zero.dim();
    result.equalityDimension = equalities.dim();
    result.inequalityDimension = inequalities.dim();
    result.assemblySeconds = seconds(start);
  }
  // Evaluate physical residuals and costs without setting up CMC manifolds.
  void summarize() {
    result.effortError = effort.error(result.values);
    result.wrenchError = wrench.error(result.values);
    result.torqueSmoothError = torqueSmooth.error(result.values);
    result.wrenchSmoothError = wrenchSmooth.error(result.values);
    result.balanceError = balance.error(result.values);
    result.accelerationError = acceleration.error(result.values);
    result.equalityL2 = equalities.violationNorm(result.values);
    result.inequalityL2 = inequalities.violationNorm(result.values);
    result.feasible = result.equalityL2 < 1e-5 && result.inequalityL2 < 1e-5;
    for (const auto& factor : rootBalance) {
      const auto native = std::dynamic_pointer_cast<NoiseModelFactor>(factor);
      const Vector6 residual = native->unwhitenedError(result.values);
      result.baseResiduals.push_back(residual);
      result.maxMomentResidual =
          std::max(result.maxMomentResidual, residual.head<3>().norm());
      result.maxForceResidual =
          std::max(result.maxForceResidual, residual.tail<3>().norm());
    }
  }
};
}  // namespace

fixed_dynamics::Result fixedKinematicsDynamicsSeed(
    const Robot& robot, const Trajectory& trajectory,
    const std::vector<Slice>& slices, const std::vector<double>& times,
    const gtsam::Values& fixedValues, const gtsam::Values& accelerationPriors,
    const std::vector<fixed_dynamics::Contact>& contacts,
    const gtsam::Vector& armature, const fixed_dynamics::Options& options) {
  if (options.softDynamics)
    throw std::invalid_argument("Linear dynamics seed requires exact dynamics");
  Problem problem(robot, trajectory, slices, times, fixedValues,
                  accelerationPriors, contacts, armature, options);
  const auto started = Clock::now();
  auto& result = problem.result;
  result.values = equalityAnchor(problem.acceleration, problem.equalities,
                                 problem.zero, problem.contactSeed);
  result.initializationSeconds = seconds(started);
  result.objectiveInitial = result.objectiveFinal =
      problem.cost.error(result.values);
  result.objectiveHistory = {result.objectiveFinal};
  problem.summarize();
  if (!result.feasible)
    throw std::runtime_error(
        "Linear dynamics seed violates exact mechanics or "
        "wrench/torque constraints");
  // No objective optimization or tangent-cone stationarity test was performed.
  result.projectedGradientNorm = std::numeric_limits<double>::infinity();
  return result;
}

fixed_dynamics::Result solveFixedKinematicsDynamics(
    const Robot& robot, const Trajectory& trajectory,
    const std::vector<Slice>& slices, const std::vector<double>& times,
    const gtsam::Values& fixedValues, const gtsam::Values& accelerationPriors,
    const std::vector<fixed_dynamics::Contact>& contacts,
    const gtsam::Vector& armature, const fixed_dynamics::Options& options) {
  using namespace gtsam;
  Problem problem(robot, trajectory, slices, times, fixedValues,
                  accelerationPriors, contacts, armature, options);
  auto start = Clock::now();
  Values initial = equalityAnchor(problem.acceleration, problem.equalities,
                                  problem.zero, problem.contactSeed);
  const double anchorViolation = problem.equalities.violationNorm(initial);
  if (anchorViolation > 1e-6)
    throw std::runtime_error("Fixed dynamics equality anchor is infeasible: " +
                             std::to_string(anchorViolation));
  problem.result.initializationSeconds = seconds(start);

  auto manifoldParams = std::make_shared<IEConstraintManifold::Params>();
  manifoldParams->affineEqualities = true;
  const auto independent = problem.independentKeys;
  manifoldParams->equalityBasisCreator =
      std::make_shared<EliminationBasisCreator>(
          [independent](const KeyVector& keys) {
            KeyVector selected;
            for (Key key : keys)
              if (independent.exists(key)) selected.push_back(key);
            return selected;
          });
  auto metric = std::make_shared<IERetractorParams>();
  metric->ensureFeasible = true;
  metric->feasibleThreshold = 1e-7;
  auto retractor = std::make_shared<CountedReducedALRetractor>(metric);
  manifoldParams->retractorCreator =
      std::make_shared<UniversalIERetractorCreator>(retractor);

  start = Clock::now();
  auto manifolds = IEOptimizer::identifyManifolds(
      problem.equalities, problem.inequalities, initial, manifoldParams);
  for (auto& [key, manifold] : manifolds) {
    try {
      IERetractionInfo info;
      // Project the equality-feasible anchor onto each actual wrench cone.
      // Acceleration coordinates remain free even in single stance. Projection
      // can adjust them while preserving exact articulated acceleration and
      // force/moment balance.
      manifold = manifold.retract(manifold.values().zeroVectors(), {}, &info);
      problem.result.reducedDimension += manifold.eBasis()->dim();
    } catch (const std::exception& error) {
      std::ostringstream message;
      message << "Fixed dynamics cone projection failed at source slice "
              << DynamicsSymbol(manifold.values().keys().front()).time()
              << " (softDynamics=" << options.softDynamics
              << "): " << error.what();
      for (size_t i = 0; i < problem.result.contactKeys.size(); ++i) {
        const Key contactKey = problem.result.contactKeys[i];
        if (!manifold.values().exists(contactKey)) continue;
        const auto& contact = contacts[i % contacts.size()];
        const Vector6 soleWrench = manifold.values().at<Vector6>(contactKey);
        const Vector faces =
            RectangularContactWrench(contact.halfLength, contact.halfWidth,
                                     contact.frictionCoefficient)
                .matrix() *
            soleWrench;
        message << "; " << contact.pose.link->name()
                << " sole [Nm,N]=" << soleWrench.transpose()
                << ", maximum cone face=" << faces.maxCoeff();
      }
      throw std::runtime_error(message.str());
    }
  }
  initial = manifolds.baseValues();
  problem.result.manifoldSeconds = seconds(start);
  problem.result.objectiveInitial = problem.cost.error(initial);

  // Some affine components (a swing leg or an arm, for example) are completely
  // determined by fixed kinematics. Remove their zero-dimensional manifold
  // coordinates before forming the global LM system: zero-width Jacobian
  // blocks are not valid Gaussian variables. This is exact substitution, not
  // a soft approximation or a separate per-slice optimization.
  Values determined;
  for (auto it = manifolds.begin(); it != manifolds.end();) {
    if (it->second.eBasis()->dim() == 0) {
      determined.insert(it->second.values());
      it = manifolds.erase(it);
    } else {
      ++it;
    }
  }
  const NonlinearFactorGraph variableCost =
      ConstVarGraph(problem.cost, determined);
  const double constantError =
      problem.cost.error(initial) - variableCost.error(initial);

  IELMParams params;
  params.iqpMaxIterations = 1000;
  params.lmParams.maxIterations = options.maxIterations;
  params.lmParams.absoluteErrorTol = 1e-8;
  params.lmParams.relativeErrorTol = 1e-8;
  IELMOptimizer optimizer(params, manifoldParams);
  start = Clock::now();
  auto& result = problem.result;
  // Skip a zero-column LM system if every component is uniquely determined.
  result.values =
      result.reducedDimension
          ? optimizer.optimizeManifolds(variableCost, manifolds, Values())
          : Values();
  result.values.insert(determined);
  result.optimizationSeconds = seconds(start);
  result.numALSolves = retractor->alSolves;
  result.numFeasibleTrials = retractor->feasibleTrials;
  result.numFeasibleProjections = retractor->feasibleProjections;
  result.iterations = optimizer.details().empty()
                          ? 0
                          : optimizer.details().back().state.iterations;
  for (const auto& iteration : optimizer.details())
    result.objectiveHistory.push_back(iteration.state.error + constantError);
  result.objectiveFinal = problem.cost.error(result.values);
  if (result.objectiveHistory.empty() ||
      std::abs(result.objectiveHistory.back() - result.objectiveFinal) > 1e-12)
    result.objectiveHistory.push_back(result.objectiveFinal);
  problem.summarize();
  if (!optimizer.details().empty()) {
    const auto& finalState = optimizer.details().back().state;
    const auto projected = IEOptimizer::projectTangentCone(
        finalState.manifolds, -1.0 * finalState.gradient);
    result.projectedGradientNorm = projected.second.norm();
  }
  result.stationary = result.feasible && result.projectedGradientNorm <= 1e-5;
  return result;
}
}  // namespace gtdynamics

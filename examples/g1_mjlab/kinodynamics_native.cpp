/* GTDynamics Copyright 2026, Georgia Tech Research Corporation. See LICENSE. */
/**
 * @file kinodynamics_native.cpp
 * @brief Assemble and solve the G1 coupled walking example with native CMC-Opt.
 * @author GTDynamics contributors
 */
#include <gtdynamics/cmcopt/ArticulatedRetractor.h>
#include <gtdynamics/cmcopt/IELMOptimizer.h>
#include <gtdynamics/constrained/internal/IndependentComponents.h>
#include <gtdynamics/dynamics/ContactWrenchFactor.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/dynamics/FixedKinematicsDynamics.h>
#include <gtdynamics/dynamics/WorldVelocityFactor.h>
#include <gtdynamics/factors/CollocationFactors.h>
#include <gtdynamics/kinematics/ContactKinematicsTwistFactor.h>
#include <gtdynamics/kinematics/PoseContactInitializer.h>
#include <gtdynamics/utils/ContactWrench.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <chrono>
#include <iostream>

#include "EqualityOnlyBaseline.h"
#include "FixedDynamicsBaseline.h"
#include "KinodynamicGauge.h"
#include "KinodynamicOrdering.h"
namespace py = pybind11;
namespace {
using namespace gtdynamics;
using namespace gtsam;
using Clock = std::chrono::steady_clock;
double elapsed(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}
template <class T>
T setting(const py::dict& settings, const char* name) {
  return settings[py::str(name)].cast<T>();
}
// Locate the floating root independently of the initialization strategy.
LinkSharedPtr floatingRoot(const Robot& robot) {
  LinkSharedPtr root;
  std::set<int> children;
  for (const auto& joint : robot.joints())
    children.insert(joint->child()->id());
  for (const auto& link : robot.links())
    if (!children.count(link->id())) {
      if (root) throw std::invalid_argument("Expected one articulated tree");
      root = link;
    }
  if (!root || root->isFixed())
    throw std::invalid_argument("Missing floating root");
  return root;
}
Values kinematicConfiguration(const Robot& robot,
                              const std::vector<Slice>& slices,
                              const Values& initialValues) {
  Values qv;
  for (const Slice& slice : slices) {
    for (const auto& link : robot.links()) {
      qv.insert(PoseKey(link->id(), slice.k),
                initialValues.at<Pose3>(PoseKey(link->id(), slice.k)));
      const Key velocity = TwistKey(link->id(), slice.k);
      qv.insert(velocity,
                initialValues.exists(velocity)
                    ? Vector(Twist(initialValues, link->id(), slice.k))
                    : Vector(Vector6::Zero()));
    }
    for (const auto& joint : robot.joints()) {
      qv.insert(JointAngleKey(joint->id(), slice.k),
                initialValues.at<double>(JointAngleKey(joint->id(), slice.k)));
      const Key velocity = JointVelKey(joint->id(), slice.k);
      qv.insert(velocity, initialValues.exists(velocity)
                              ? initialValues.at<double>(velocity)
                              : 0.0);
    }
  }
  return qv;
}

py::dict solve(
    const std::shared_ptr<Robot>& ownedRobot, const Trajectory& trajectory,
    const std::vector<Slice>& slices, const std::vector<double>& times,
    const gtsam::Values& initialValues,
    const std::vector<gtsam::NonlinearFactorGraph>& measurementGroups,
    const std::vector<std::string>& measurementNames,
    const std::vector<fixed_dynamics::Contact>& contacts,
    const gtsam::Vector& armature, const py::dict& settings) {
  if (!ownedRobot) throw std::invalid_argument("Missing robot");
  const Robot& robot = *ownedRobot;
  const auto assemblyStart = Clock::now();
  if (measurementGroups.size() != measurementNames.size() ||
      slices.size() < 2 || !setting<size_t>(settings, "maxIterations") ||
      setting<size_t>(settings, "retractionThreads") < 1)
    throw std::invalid_argument(
        "Kinodynamics requires named groups and >=2 slices");
  for (double sigma : {setting<double>(settings, "poseRotationSigmaDegrees"),
                       setting<double>(settings, "poseTranslationSigma"),
                       setting<double>(settings, "twistAngularSigmaDegrees"),
                       setting<double>(settings, "twistLinearSigma"),
                       setting<double>(settings, "jointPositionSigmaDegrees"),
                       setting<double>(settings, "jointVelocitySigmaDegrees"),
                       setting<double>(settings, "stanceVelocitySigma")})
    if (!(sigma > 0) || !std::isfinite(sigma))
      throw std::invalid_argument(
          "Kinodynamics sigmas must be finite and positive");
  if (std::none_of(slices.begin(), slices.end(), [&](const Slice& slice) {
        return slice.k == setting<size_t>(settings, "gaugeSlice");
      }))
    throw std::invalid_argument("The gauge slice must be in the trajectory");
  const auto root = floatingRoot(robot);
  const std::string initialization =
      setting<std::string>(settings, "initialization");
  if (initialization != "fk" && initialization != "contact" &&
      initialization != "contact-gp-dynamics")
    throw std::invalid_argument("Unknown kinodynamics initialization");
  double totalMass = 0.0;
  for (const auto& link : robot.links()) totalMass += link->mass();
  const Values qv = kinematicConfiguration(robot, slices, initialValues);
  Values seed = qv;
  NonlinearFactorGraph exact, effort, wrench, torqueSmooth, wrenchSmooth,
      acceleration;
  NonlinearInequalityConstraints inequalities;
  KeySet independentKeys;
  std::vector<Key> contactKeys, torqueKeys;
  std::vector<size_t> jointIds;
  // Per-slice mechanics. Scheduling and regularization are experiment choices;
  // the equations come from the existing library factor builders.
  if (slices.empty() || slices.size() != times.size() || contacts.empty())
    throw std::invalid_argument(
        "Dynamics requires matching slices/times and contacts");
  if (!setting<Vector3>(settings, "gravity").allFinite())
    throw std::invalid_argument("Invalid gravity");
  if (!armature.allFinite())
    throw std::invalid_argument("Armature must be finite");
  for (double sigma :
       {setting<double>(settings, "effortSigma"),
        setting<double>(settings, "wrenchSigma"),
        setting<double>(settings, "torqueSmoothSigma"),
        setting<double>(settings, "wrenchSmoothSigma"),
        setting<double>(settings, "baseAngularAccelerationSigmaDegrees"),
        setting<double>(settings, "baseLinearAccelerationSigma"),
        setting<double>(settings, "jointAccelerationSigmaDegrees")})
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
  for (const auto& joint : joints) jointIds.push_back(joint->id());
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
      throw std::invalid_argument("Dynamics contact transform must be finite");
  }

  const auto tracked = contactPoseFrames(trajectory);
  const double radiansPerDegree = std::acos(-1.0) / 180.0;
  Vector6 baseAccelerationSigmas;
  baseAccelerationSigmas << Vector3::Constant(
      setting<double>(settings, "baseAngularAccelerationSigmaDegrees") *
      radiansPerDegree),
      Vector3::Constant(
          setting<double>(settings, "baseLinearAccelerationSigma"));
  const auto baseAccelerationNoise =
      noiseModel::Diagonal::Sigmas(baseAccelerationSigmas);
  const auto jointAccelerationNoise = noiseModel::Isotropic::Sigma(
      1, setting<double>(settings, "jointAccelerationSigmaDegrees") *
             radiansPerDegree);
  DynamicsParameters parameters(1.0);
  parameters.gravity = setting<Vector3>(settings, "gravity");
  const Dynamics dynamics(parameters);
  const Mechanics mechanicsBuilder(parameters);
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
      contactKeys.push_back(soleKey);
      seed.insert(soleKey, Vector(Vector6::Zero()));
      seed.insert(comKey, Vector(Vector6::Zero()));
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
        const auto cone = RectangularContactWrench(
            contact.halfLength, contact.halfWidth, contact.frictionCoefficient);
        for (const auto& inequality : cone.constraints(soleKey))
          inequalities.push_back(inequality);
      } else {
        exact.emplace_shared<PriorFactor<Vector6>>(soleKey, Vector6::Zero(),
                                                   noiseModel::Unit::Create(6));
      }
      const double length = std::sqrt(contact.halfLength * contact.halfWidth);
      Vector6 sigmas;
      sigmas << Vector3::Constant(setting<double>(settings, "wrenchSigma") *
                                  length),
          Vector3::Constant(setting<double>(settings, "wrenchSigma"));
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
        sigmas *= setting<double>(settings, "wrenchSmoothSigma") /
                  setting<double>(settings, "wrenchSigma") *
                  std::sqrt(times[i] - times[i - 1]);
        wrenchSmooth.emplace_shared<BetweenFactor<Vector6>>(
            ContactWrenchKey(id, c, slices[i - 1].k), soleKey, Vector6::Zero(),
            noiseModel::Diagonal::Sigmas(sigmas));
      }
    }
    if (active.empty())
      throw std::invalid_argument(
          "This experiment requires support at every slice");

    if (initialization == "fk") {
      // Share the world support force across stance feet. Each wrench has zero
      // moment at its sole origin; transform its force into sole axes and then
      // use the library's dual-adjoint map for the COM wrench and lever arm.
      // This is only a seed: restoration resolves moments, friction and
      // torques.
      const Vector3 worldForce =
          -totalMass * parameters.gravity.value() / active.size();
      for (size_t c : active) {
        const auto& contact = contacts[c];
        const int id = contact.pose.link->id();
        const Pose3 worldContact =
            qv.at<Pose3>(PoseKey(id, k)) * contact.pose.comTcontact;
        Vector6 soleWrench = Vector6::Zero();
        soleWrench.tail<3>() = worldContact.rotation().unrotate(worldForce);
        seed.update(ContactWrenchKey(id, c, k), Vector(soleWrench));
        seed.update(
            DynamicsSymbol::LinkJointSymbol("ec", id, c, k),
            Vector(contactWrenchToCom(contact.pose.comTcontact, soleWrench)));
      }
    }

    // Root Newton-Euler balance determines one active six-axis contact
    // wrench. All other active contact wrenches are independent coordinates.
    for (size_t a = 1; a < active.size(); ++a)
      independentKeys.insert(
          ContactWrenchKey(contacts[active[a]].pose.link->id(), active[a], k));

    for (const auto& link : robot.links()) {
      for (const auto& joint : link->joints()) {
        const Key key = WrenchKey(link->id(), joint->id(), k);
        seed.insert(key, Vector(Vector6::Zero()));
      }

      const Key accelerationKey = TwistAccelKey(link->id(), k);
      seed.insert(accelerationKey, Vector(Vector6::Zero()));
      if (link == root) {
        independentKeys.insert(accelerationKey);
        acceleration.emplace_shared<PriorFactor<Vector6>>(
            accelerationKey, Vector6::Zero(), baseAccelerationNoise);
      }
    }
    exact.add(dynamics.wrenchBalanceFactors(Slice(k), robot, external));
    exact.add(dynamics.aFactors(Slice(k), robot));
    exact.add(mechanicsBuilder.wrenchEquivalenceFactors(Slice(k), robot));
    exact.add(mechanicsBuilder.torqueFactors(Slice(k), robot, armature));
    for (const auto& joint : joints) {
      const int id = joint->id();

      const Key accelerationKey = JointAccelKey(id, k);
      seed.insert(accelerationKey, 0.0);
      independentKeys.insert(accelerationKey);
      acceleration.emplace_shared<PriorFactor<double>>(accelerationKey, 0.0,
                                                       jointAccelerationNoise);
      const Key torqueKey = TorqueKey(id, k);
      torqueKeys.push_back(torqueKey);
      seed.insert(torqueKey, 0.0);
      effort.emplace_shared<PriorFactor<double>>(
          torqueKey, 0.0,
          noiseModel::Isotropic::Sigma(
              1, setting<double>(settings, "effortSigma")));
      if (i)
        torqueSmooth.emplace_shared<BetweenFactor<double>>(
            TorqueKey(id, slices[i - 1].k), torqueKey, 0.0,
            noiseModel::Isotropic::Sigma(
                1, setting<double>(settings, "torqueSmoothSigma") *
                       std::sqrt(times[i] - times[i - 1])));
    }
  }
  auto groups = measurementGroups;
  auto names = measurementNames;
  NonlinearFactorGraph poseCollocation, velocityCollocation, stanceVelocity;
  Vector6 poseSigmas, twistSigmas;
  poseSigmas << Vector3::Constant(
      setting<double>(settings, "poseRotationSigmaDegrees") * radiansPerDegree),
      Vector3::Constant(setting<double>(settings, "poseTranslationSigma"));
  twistSigmas << Vector3::Constant(
      setting<double>(settings, "twistAngularSigmaDegrees") * radiansPerDegree),
      Vector3::Constant(setting<double>(settings, "twistLinearSigma"));
  const auto poseNoise = noiseModel::Diagonal::Sigmas(poseSigmas);
  const auto twistNoise = noiseModel::Diagonal::Sigmas(twistSigmas);
  const auto qNoise = noiseModel::Isotropic::Sigma(
      1, setting<double>(settings, "jointPositionSigmaDegrees") *
             radiansPerDegree);
  const auto vNoise = noiseModel::Isotropic::Sigma(
      1, setting<double>(settings, "jointVelocitySigmaDegrees") *
             radiansPerDegree);
  KinematicsParameters kp;
  kp.p_cost_model = kp.v_cost_model = noiseModel::Unit::Create(6);
  const Kinematics kinematics(kp);
  KeySet independent = independentKeys;
  const Key gaugeKey =
      PoseKey(root->id(), setting<size_t>(settings, "gaugeSlice"));
  const auto gauge = g1_example::gaugeFactors(gaugeKey, qv.at<Pose3>(gaugeKey));
  for (size_t i = 0; i < slices.size(); ++i) {
    const size_t k = slices[i].k;
    const Key poseKey = PoseKey(root->id(), k),
              twistKey = TwistKey(root->id(), k);
    seed.insert(LinearVelocityKey(root->id(), k), Vector(Vector3::Zero()));
    exact.emplace_shared<WorldVelocityFactor>(poseKey, twistKey,
                                              LinearVelocityKey(root->id(), k),
                                              noiseModel::Unit::Create(3));
    independent.insert(twistKey);
    independent.insert(poseKey);
    exact.add(kinematics.constraints(slices[i], robot).penaltyGraph());
    exact.add(kinematics.twistObjectives(slices[i], robot));
    for (const auto& joint : robot.joints()) {
      const int id = joint->id();
      independent.insert(JointAngleKey(id, k));
      independent.insert(JointVelKey(id, k));
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
            noiseModel::Isotropic::Sigma(
                3, setting<double>(settings, "stanceVelocitySigma")),
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
  // Retain every completed stage of initialization. Graph assembly supplies
  // defaults for absent keys only; GP motion and solved dynamics must survive.
  for (Key key : initialValues.keys()) {
    if (!seed.exists(key)) {
      seed.insert(key, initialValues.at(key));
      continue;
    }
    // Native FK stores fixed-size twists; graph assembly uses dynamic vectors.
    // Normalize those representations before Values::update's strict type
    // check.
    const auto label = DynamicsSymbol(key).label();
    if (label == "V" || label == "A" || label == "F" || label == "C" ||
        label == "ec")
      seed.update(key, Vector(initialValues.at<Vector6>(key)));
    else if (label == "vW")
      seed.update(key, Vector(initialValues.at<Vector3>(key)));
    else
      seed.update(key, initialValues.at(key));
  }
  groups.insert(groups.end(),
                {poseCollocation, velocityCollocation, stanceVelocity, effort,
                 wrench, torqueSmooth, wrenchSmooth, acceleration});
  names.insert(names.end(),
               {"Configuration collocation", "Velocity collocation",
                "Stance velocity", "Torque effort", "Contact wrench",
                "Torque rate", "Wrench rate", "Acceleration regularization"});
  groups.push_back(gauge);
  names.push_back("Translation/yaw gauge");
  NonlinearFactorGraph cost;
  for (const auto& group : groups) cost.add(group);
  // Keep the omitted bounds only for diagnostics, never for retraction.
  const auto originalInequalities = inequalities;
  const bool equalityOnly = setting<bool>(settings, "equalityOnly");
  if (equalityOnly) inequalities = NonlinearInequalityConstraints();
  const auto equalities = NonlinearEqualityConstraints::FromCostGraph(exact);
  py::dict result;
  result["initialValues"] = seed;
  result["initialEqualityL2"] = equalities.violationNorm(seed);
  result["variableDimension"] = seed.dim();
  result["equalityDimension"] = equalities.dim();
  result["inequalityDimension"] = inequalities.dim();
  result["equalityOnly"] = equalityOnly;
  result["originalInequalityDimension"] = originalInequalities.dim();
  result["errorNames"] = names;
  result["contactKeys"] = contactKeys;
  result["torqueKeys"] = torqueKeys;
  result["jointIds"] = jointIds;
  result["assemblySeconds"] = elapsed(assemblyStart);
  std::vector<Vector> errorHistory;
  std::vector<double> wallSeconds;
  size_t iterations = 0, reducedDimension = 0;
  auto manifoldParams = std::make_shared<IEConstraintManifold::Params>();
  manifoldParams->equalityBasisCreator =
      std::make_shared<EliminationBasisCreator>(
          [independent](const KeyVector& keys) {
            KeyVector selected;
            for (Key key : keys)
              if (independent.exists(key)) selected.push_back(key);
            return selected;
          });
  auto projection = std::make_shared<IERetractorParams>();
  projection->feasibleThreshold =
      setting<double>(settings, "projectionFeasibilityTolerance");
  projection->useVaryingSigma = true;
  projection->metricSigmas = std::make_shared<VectorValues>();
  for (Key key : independentKeys) {
    Vector sigmas;
    const DynamicsSymbol symbol(key);
    if (symbol.label() == "A") {
      sigmas = Vector6::Constant(
          setting<double>(settings,
                          "projectionAngularAccelerationSigmaDegrees") *
          radiansPerDegree);
      sigmas.tail<3>().setConstant(
          setting<double>(settings, "projectionLinearAccelerationSigma"));
    } else if (symbol.label() == "a") {
      sigmas = Vector1(
          setting<double>(settings, "projectionJointAccelerationSigmaDegrees") *
          radiansPerDegree);
    } else {
      sigmas =
          Vector6::Constant(setting<double>(settings, "projectionForceSigma"));
      sigmas.head<3>().setConstant(
          setting<double>(settings, "projectionMomentSigma"));
    }
    projection->metricSigmas->insert(key, sigmas);
  }
  ReducedALRetractor::Settings projectionSettings;
  projectionSettings.al.absoluteStationarityTolerance =
      setting<double>(settings, "projectionStationarityTolerance");
  projectionSettings.feasibilityTolerance = projection->feasibleThreshold;
  projectionSettings.activeTolerance =
      setting<double>(settings, "projectionActiveTolerance");
  manifoldParams->retractorCreator =
      std::make_shared<ArticulatedRetractorCreator>(ownedRobot, root->name(),
                                                    independentKeys, projection,
                                                    projectionSettings);
  auto start = Clock::now();
  IEOptimizer::TypeFactory makeType;
  if (equalityOnly)
    makeType = [](const NonlinearEqualityConstraints& equalities,
                  const NonlinearInequalityConstraints&, const Values& values,
                  const IEConstraintManifold::Params::shared_ptr& params) {
      return std::make_shared<EConstraintManifold>(equalities, values, *params);
    };
  auto components = IEOptimizer::identifyConnectedComponents(
      equalities, inequalities, seed, manifoldParams, makeType);
  auto& points = components.points;
  const auto componentKeys = pointKeys(points);
  gtdynamics::internal::forIndependentComponents(
      componentKeys.size(), setting<size_t>(settings, "retractionThreads"),
      [&](size_t i) {
        auto& point = points.at(componentKeys[i]);
        point = point.retract(point.values().zeroVectors());
      });
  for (const auto& [key, point] : points)
    reducedDimension += point.eBasis()->dim();
  Values initial = ambientValues(points);
  const Values unconstrained =
      IEOptimizer::identifyUnconstrainedValues(equalities, inequalities, seed);
  initial.insert(unconstrained);
  result["initializationSeconds"] = elapsed(start);
  result["reducedDimension"] = reducedDimension;
  result["objectiveInitial"] = cost.error(initial);
  IELMParams params;
  const auto linearSolver = setting<std::string>(settings, "linearSolver");
  if (linearSolver == "gaussian")
    params.linearSolver = IELMParams::LinearSolver::Gaussian;
  else if (linearSolver == "qp")
    params.linearSolver = IELMParams::LinearSolver::QP;
  else if (linearSolver != "automatic")
    throw std::invalid_argument("Unknown linear solver: " + linearSolver);
  const auto qpBackend = setting<std::string>(settings, "qpBackend");
  if (qpBackend == "proxqp")
    params.qpBackend = IELMParams::QpBackend::ProxQP;
  else if (qpBackend != "gtsam")
    throw std::invalid_argument("Unknown QP backend: " + qpBackend);
  params.qpMaxIterations = setting<size_t>(settings, "qpMaxIterations");
  params.qpUseProjectedGradientInitialization =
      setting<bool>(settings, "qpUseProjectedGradientInitialization");
  params.qpUseFeasibleStep = setting<bool>(settings, "qpUseFeasibleStep");
  params.historyWindow = setting<size_t>(settings, "historyWindow");
  params.retractionThreads = setting<size_t>(settings, "retractionThreads");
  params.lmParams.maxIterations = setting<size_t>(settings, "maxIterations");
  params.lmParams.relativeErrorTol =
      setting<double>(settings, "relativeErrorTol");
  params.lmParams.absoluteErrorTol =
      setting<double>(settings, "absoluteErrorTol");
  params.lmParams.lambdaInitial = setting<double>(settings, "lambdaInitial");
  params.lmParams.setDiagonalDamping(
      setting<bool>(settings, "diagonalDamping"));
  std::vector<std::string> orderingKeys;
  if (setting<bool>(settings, "qpUseTemporalOrdering")) {
    params.lmParams.ordering = g1_example::chronologicalOrdering(cost, points, unconstrained, slices.front().k);
    for (Key key : *params.lmParams.ordering)
      orderingKeys.push_back(std::to_string(key));
  }
  result["qpOrderingKeys"] = orderingKeys;
  const std::string equalityBaseline = settings.contains("equalityOnlyBaseline")
      ? setting<std::string>(settings, "equalityOnlyBaseline") : "none";
  if (equalityBaseline != "none" &&
      (!equalityOnly || params.retractionThreads != 1 ||
       equalityBaseline != "manifold-lm"))
    throw std::invalid_argument("Equality baseline requires empty inequalities and one worker");
  result["equalityOnlyBaseline"] = equalityBaseline;
  const BasisKeyFunction select = [independent](const KeyVector& keys) {
    KeyVector selected;
    for (Key key : keys) if (independent.exists(key)) selected.push_back(key);
    return selected;
  };
  const auto optimizeStart = Clock::now();
  // Each backend owns elimination. ProxQP uses the supplied order for packing.
  params.lmParams.verbosityLM = LevenbergMarquardtParams::SUMMARY;
  params.iterationCallback = [&](const IELMState& state) {
    const Values values = state.baseValues();
    Vector errors(groups.size());
    for (size_t i = 0; i < groups.size(); ++i)
      errors(i) = groups[i].error(values);
    errorHistory.push_back(errors);
    wallSeconds.push_back(elapsed(optimizeStart));
    iterations = state.iterations;
    std::cout << "Kinodynamics iteration " << state.iterations
              << ": error=" << state.error << ", seconds=" << wallSeconds.back()
              << std::endl;
  };
  Values values;
  IELMOptimizer::Timings timings;
  if (equalityBaseline == "manifold-lm") {
    const auto prepareStart = Clock::now();
    const auto problem = g1_example::manifoldLmProblem(
        cost, points, unconstrained, manifoldParams->equalityBasisCreator, select);
    result["manifoldLmPreparationSeconds"] = elapsed(prepareStart);
    LMManifoldOptimizer optimizer(ManifoldOptimizerParameters{}, params.lmParams);
    values = optimizer.optimize(cost, problem);
    iterations = optimizer.details().back().state.iterations;
    // Manifold LM has no callback hook. Evaluate the same stored-state diagnostics
    // here, including their cost in the reported optimization wall time.
    for (const auto& iteration : optimizer.details()) {
      const auto ambient = iteration.state.baseValues();
      Vector errors(groups.size());
      for (size_t i = 0; i < groups.size(); ++i)
        errors(i) = groups[i].error(ambient);
      errorHistory.push_back(errors);
      timings.gaussianSolves += iteration.trials.size();
    }
  } else {
    IELMOptimizer optimizer(params, manifoldParams);
    values = optimizer.optimize(cost, points, unconstrained);
    timings = optimizer.timings();
    if (equalityBaseline == "none") result["manifoldLmPreparationSeconds"] = 0.;
  }
  result["values"] = values;
  result["optimizationSeconds"] = elapsed(optimizeStart);
  result["linearizationSeconds"] = timings.linearizationSeconds;
  result["linearSolveSeconds"] = timings.linearSolveSeconds;
  result["retractionSeconds"] = timings.retractionSeconds;
  if (equalityBaseline == "manifold-lm") {
    // Manifold LM exposes total trial times, not these phase timings.
    result["linearizationSeconds"] = py::none();
    result["linearSolveSeconds"] = py::none();
    result["retractionSeconds"] = py::none();
  }
  result["gaussianSolves"] = timings.gaussianSolves;
  result["qpSolves"] = timings.qpSolves;
  result["objectiveFinal"] = cost.error(values);
  const double equalityL2 = equalities.violationNorm(values);
  const double inequalityL2 = inequalities.violationNorm(values);
  result["equalityL2"] = equalityL2;
  result["inequalityL2"] = inequalityL2;
  result["originalInequalityL2"] = originalInequalities.violationNorm(values);
  result["feasible"] = equalityL2 < 1e-5 && inequalityL2 < 1e-5;
  result["errorHistory"] = errorHistory;
  result["wallSeconds"] = wallSeconds;
  result["iterations"] = iterations;
  return result;
}
}  // namespace
PYBIND11_MODULE(_g1_kinodynamics, module) {
  py::module_::import("gtsam");
  py::module_::import("gtdynamics");
#ifdef GTDYNAMICS_WITH_PROXQP
  module.attr("has_proxqp") = true;
#else
  module.attr("has_proxqp") = false;
#endif
  module.def("solve_fixed_dynamics_manifold_lm", [](
      const Robot& robot, const Trajectory& trajectory,
      const std::vector<Slice>& slices, const std::vector<double>& times,
      const Values& fixed, const Values& priors,
      const std::vector<fixed_dynamics::Contact>& contacts, const Vector& armature,
      const fixed_dynamics::Options& options) {
    auto output = g1_example::solveManifoldLmFixedDynamics(
        robot, trajectory, slices, times, fixed, priors, contacts, armature, options);
    return py::make_tuple(output.result, output.manifoldLmPreparationSeconds);
  });
  module.def("solve", &solve, py::arg("robot"), py::arg("trajectory"),
             py::arg("slices"), py::arg("times"), py::arg("initial_values"),
             py::arg("measurement_groups"), py::arg("measurement_names"),
             py::arg("contacts"), py::arg("armature"), py::arg("settings"));
}

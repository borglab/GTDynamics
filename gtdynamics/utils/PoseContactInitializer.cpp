/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file PoseContactInitializer.cpp
 * @brief Initialize floating-root poses from shared sole-pose landmarks.
 * @author GTDynamics contributors
 */
#include <gtdynamics/factors/ContactPointFactor.h>
#include <gtdynamics/kinematics/ContactHeightFactor.h>
#include <gtdynamics/kinematics/ContactPoseEqualityFactor.h>
#include <gtdynamics/utils/ContactInitializationFactors.h>
#include <gtdynamics/utils/Initializer.h>
#include <gtdynamics/utils/PoseContactInitializer.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <set>

namespace gtdynamics {
namespace {

void require(bool condition, const std::string& message) {
  if (!condition) throw std::invalid_argument("Contact poses: " + message);
}

// Copy only this slice's joint measurements. FK also expects joint velocities;
// use recorded velocities when available and zero otherwise. These velocities
// do not become measurements or unknowns in the contact-pose optimization.
gtsam::Values measuredSlice(const Robot& robot, const gtsam::Values& measured,
                            size_t k) {
  gtsam::Values values;
  for (const auto& joint : robot.joints()) {
    const auto q = JointAngleKey(joint->id(), k);
    const double angle = measured.at<double>(q);
    require(std::isfinite(angle), "nonfinite joint measurement");
    values.insert(q, angle);

    const auto v = JointVelKey(joint->id(), k);
    values.insert(v, measured.exists(v) ? measured.at<double>(v) : 0.0);
  }
  return values;
}

}  // namespace

std::map<size_t, std::vector<TrackedPoseContact>> contactPoseFrames(
    const Trajectory& trajectory) {
  std::map<size_t, std::vector<TrackedPoseContact>> frames;
  std::vector<TrackedPoseContact> previous;
  size_t next = 0, previousEnd = 0;
  bool first = true;

  // Associate contacts BEFORE selecting slices. A stance can continue through
  // several phases while the other foot lifts or lands. Conversely, skipping a
  // swing phase during sampling must not join two distinct footholds.
  for (const auto& phase : trajectory.phases()) {
    require(phase.k_start <= phase.k_end &&
                phase.k_end <= size_t(std::numeric_limits<int>::max()),
            "invalid phase bounds");
    require(first ||
                (phase.k_start >= previousEnd &&
                 phase.k_start <= previousEnd + 1 && phase.k_end > previousEnd),
            "phases must be ordered and contiguous");
    require(bool(phase.constraintSpec()), "missing phase specification");

    std::vector<TrackedPoseContact> current;
    for (const auto& contact : phase.constraintSpec()->contactPoses()) {
      require(bool(contact.link) && contact.comTcontact.matrix().allFinite(),
              "invalid frame");
      require(std::none_of(current.begin(), current.end(),
                           [&](const auto& c) {
                             return c.contact.link == contact.link;
                           }),
              "one contact frame per link per phase required");

      // Equality includes the link, material-frame transform, and ground role.
      // Only the immediately preceding phase can carry a stance ID forward;
      // disappearance followed by recontact always allocates a fresh ID.
      auto match = std::find_if(
          previous.begin(), previous.end(),
          [&](const auto& c) { return c.contact.equals(contact, 0); });
      current.push_back(
          {contact, match == previous.end() ? next++ : match->stance});
    }

    // Phases may share an endpoint. At that slice, retain the union of active
    // stances without duplicating a stance that continues through the boundary.
    for (size_t k = phase.k_start; k <= phase.k_end; ++k) {
      auto& active = frames[k];
      for (const auto& contact : current)
        if (std::none_of(active.begin(), active.end(), [&](const auto& c) {
              return c.stance == contact.stance;
            }))
          active.push_back(contact);
    }

    previous = current;
    previousEnd = phase.k_end;
    first = false;
  }

  require(!frames.empty(), "empty trajectory");
  return frames;
}

std::vector<Slice> contactInitializationSlices(const Trajectory& trajectory,
                                               double dt,
                                               double maxSliceInterval,
                                               size_t contactBuffer,
                                               double longPhaseSeconds) {
  require(std::isfinite(dt) && dt > 0 && std::isfinite(maxSliceInterval) &&
              maxSliceInterval >= dt && std::isfinite(longPhaseSeconds) &&
              longPhaseSeconds > 0,
          "invalid contact sampling interval");
  contactPoseFrames(
      trajectory);  // Validate phase bounds and contact identities.

  std::set<size_t> selected;
  for (const auto& phase : trajectory.phases()) {
    if (phase.constraintSpec()->contactPoses().empty()) continue;
    const size_t width = phase.k_end - phase.k_start;
    if ((width + 1) * dt <=
        std::min(longPhaseSeconds, maxSliceInterval) + 1e-12) {
      // Preserve even a one-sample support phase, including a short handover.
      selected.insert(phase.k_start + width / 2);
      continue;
    }

    // Buffer each phase inward, capping the margin to retain a nonempty
    // interior. Stance identity is still determined on the original trajectory.
    const size_t buffer = std::min(contactBuffer, width / 2);
    const size_t first = phase.k_start + buffer, last = phase.k_end - buffer;
    const size_t span = last - first;
    if (span == 0) {
      selected.insert(first);
      continue;
    }
    const size_t maxSteps = std::max<size_t>(
        1, size_t(std::min(double(span),
                           std::floor(maxSliceInterval / dt + 1e-12))));
    const size_t intervals = (span + maxSteps - 1) / maxSteps;
    // Integer nearest rounding spreads gaps evenly; every gap <= maxSteps.
    // Ties choose the later sample. No interpolated sample is synthesized.
    for (size_t i = 0; i <= intervals; ++i)
      selected.insert(first + (i * span + intervals / 2) / intervals);
  }

  std::vector<Slice> result;
  for (size_t k : selected) result.emplace_back(k);
  return result;
}

gtsam::Values Initializer::InitializeSolutionContactPoses(
    const Robot& robot, const std::string& rootLinkName,
    const gtsam::Pose3& initialPose, const gtsam::Values& measuredJoints,
    const Trajectory& trajectory, double dt,
    const pose_contact_init::Options& options,
    pose_contact_init::Result* diagnostics, size_t contactBuffer) const {
  using gtsam::Pose3;
  using gtsam::Values;
  using Clock = std::chrono::steady_clock;

  const auto begin = Clock::now();

  // Pose3 residuals require angular noise in radians. Callers specifying
  // rotation standard deviations in degrees must convert them before populating
  // Options.
  require(std::isfinite(dt) && dt > 0, "positive source dt required");
  for (double sigma :
       {options.rotationSigma, options.translationSigma, options.groundSigma,
        options.normalSigma, options.uprightSigma,
        options.betweenTranslationSigma, options.betweenRotationSigma})
    require(std::isfinite(sigma) && sigma > 0,
            "positive finite noise scales required");
  require(
      std::isfinite(options.maxSliceInterval) && options.maxSliceInterval > 0,
      "positive finite sampling interval required");
  require(options.maxIterations > 0 && initialPose.matrix().allFinite(),
          "invalid optimizer options or gauge");
  const auto root = robot.link(rootLinkName);
  require(bool(root) && !root->isFixed(), "floating root required");

  // Associate footholds on the whole trajectory before sampling. Skipping
  // slices must neither split continuing stances nor merge separate footholds.
  const auto frames = contactPoseFrames(trajectory);
  std::map<size_t, PoseOnLink> definitions;
  for (const auto& [k, contacts] : frames)
    for (const auto& c : contacts) {
      require(robot.link(c.contact.link->name()) == c.contact.link,
              "foreign contact link");
      definitions.emplace(c.stance, c.contact);
    }

  // Select source indices, not newly synthesized times. Explicit selection has
  // already applied its policy (including phase buffering), so
  // it must not be buffered a second time.
  std::set<size_t> selected;
  if (!options.selectedSlices.empty()) {
    require(contactBuffer == 0, "explicit slices cannot be buffered again");
    for (const auto& slice : options.selectedSlices) {
      require(frames.count(slice.k) && !frames.at(slice.k).empty() &&
                  (selected.empty() || slice.k > *selected.rbegin()),
              "explicit slices must be ordered, unique, and have contact");
      selected.insert(slice.k);
    }
  } else {
    for (const auto& slice : contactInitializationSlices(
             trajectory, dt, std::max(dt, options.maxSliceInterval),
             contactBuffer))
      selected.insert(slice.k);
  }
  require(!selected.empty(), "no active contact frames");

  // Freeze encoder-derived geometry into observations Z_ic = root_T_contact.
  // Setting the root COM pose to identity makes FK produce root-relative link
  // poses; PoseOnLink then composes each link's COM_T_contact offset. No world
  // reference poses, gyro data, or interpolated states enter these
  // observations.
  struct Observation {
    size_t stance;
    Pose3 local;
  };
  std::vector<std::vector<Observation>> observations;
  pose_contact_init::Result result;
  for (size_t k : selected) {
    auto known = measuredSlice(robot, measuredJoints, k);
    InsertPose(&known, root->id(), k, Pose3());
    const auto fk = robot.forwardKinematics(known, k, rootLinkName);
    std::vector<Observation> row;
    for (const auto& c : frames.at(k))
      row.push_back({c.stance, c.contact.predict(fk, k)});
    observations.push_back(row);
    result.selectedSlices.emplace_back(k);
    result.times.push_back(k * dt);
  }

  // Sparse selection can omit entire footholds. Diagnostics use compact vector
  // indices, while graph keys below retain the original stance IDs. Sorting
  // both maps by stance ID keeps contactFrames, contactPoses, and observations
  // aligned.
  std::map<size_t, size_t> contactIndices;
  for (const auto& row : observations)
    for (const auto& observation : row)
      contactIndices.emplace(observation.stance, 0);
  for (auto& [stance, index] : contactIndices) {
    index = result.contactFrames.size();
    result.contactFrames.push_back(definitions.at(stance));
  }
  for (size_t i = 0; i < observations.size(); ++i)
    for (const auto& observation : observations[i])
      result.observations.push_back(
          {i, contactIndices.at(observation.stance), observation.local});

  // Root keys preserve source slice indices. Each stance gets one shared world
  // contact-pose landmark, regardless of how many selected slices observe it.
  const auto rootKey = [&](size_t i) {
    return PoseKey(root->id(), result.selectedSlices.at(i).k);
  };
  const auto contactKey = [](size_t stance) {
    return gtsam::Symbol('c', stance);
  };

  std::map<size_t, Pose3> roots{{0, initialPose}}, contacts;
  // Last already initialized observation of each stance: root index and FK.
  std::map<size_t, std::pair<size_t, Pose3>> contactReferences;

  // Construct a seed by propagating X_i * Z_ic = C_c through the bipartite
  // root/stance graph. A known landmark gives X_i = C_c * inverse(Z_ic); a
  // known root gives C_c = X_i * Z_ic. Repeated passes allow a later
  // double-support slice to connect roots that could not be reached on the
  // first pass. The first available observation wins during seeding: this is
  // not a weighted fit, so changing observation sigmas does not change this
  // propagation seed.
  bool changed = true;
  while (changed) {
    changed = false;
    for (size_t i = 0; i < observations.size(); ++i) {
      if (!roots.count(i))
        for (const auto& o : observations[i])
          if (contacts.count(o.stance)) {
            Pose3 pose = contacts.at(o.stance) * o.local.inverse();
            if (options.initializeUpright) {
              const auto& [previous, local] = contactReferences.at(o.stance);
              const auto relative = local * o.local.inverse();
              // Only yaw is carried from this relative root observation.
              // Translation still satisfies the shared contact origin exactly.
              // No IMU or later reference poses enter these starting values.
              const auto rotation = roots.at(previous).rotation() *
                                    gtsam::Rot3::Rz(relative.rotation().yaw());
              pose =
                  Pose3(rotation, contacts.at(o.stance).translation() -
                                      rotation.rotate(o.local.translation()));
            }
            roots.emplace(i, pose);
            changed = true;
            break;
          }

      if (roots.count(i))
        for (const auto& o : observations[i]) {
          if (contacts.emplace(o.stance, roots.at(i) * o.local).second)
            changed = true;
          contactReferences.insert_or_assign(o.stance,
                                             std::make_pair(i, o.local));
        }
    }

    // A disconnected support component has no pose relative to the anchor.
    // Only explicitly enabled motion priors allow us to bridge it; the seed
    // assumes zero motion across that bridge, matching the factors added below.
    if (!changed && roots.size() < observations.size() &&
        options.addBetweenFactors) {
      for (size_t i = 1; i < observations.size(); ++i)
        if (!roots.count(i) && roots.count(i - 1)) {
          roots.emplace(i, roots.at(i - 1));
          changed = true;
          break;
        }
    }
  }

  require(roots.size() == observations.size(),
          "disconnected contact graph; enable motion priors explicitly");

  // Pose3 residual order is rotation followed by translation. The translational
  // part uses Pose3 local coordinates, not a separate world-XYZ subtraction.
  gtsam::Vector6 sigmas;
  sigmas << options.rotationSigma, options.rotationSigma, options.rotationSigma,
      options.translationSigma, options.translationSigma,
      options.translationSigma;
  const auto noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
  gtsam::NonlinearFactorGraph graph;
  Values seed;

  // Fix the first SELECTED root exactly by excluding it from the free
  // variables. The supplied initialPose is the gauge even when the first source
  // slice was removed by sampling. All contact landmarks remain free pose
  // variables.
  for (size_t i = 1; i < observations.size(); ++i)
    seed.insert(rootKey(i), roots.at(i));
  for (const auto& [stance, pose] : contacts)
    seed.insert(contactKey(stance), pose);

  // For each observation, penalize Local(X_i * Z_ic, C_c). Reusing C_c across a
  // stance softly enforces a stationary SIX-dimensional contact pose, including
  // orientation. A support label alone does not establish this rigid-foot
  // assumption: rocking feet can produce incompatible observations.
  // Substituting the fixed first root turns its factors into landmark priors.
  for (size_t i = 0; i < observations.size(); ++i)
    for (const auto& o : observations[i]) {
      if (i == 0)
        graph.emplace_shared<gtsam::PriorFactor<Pose3>>(
            contactKey(o.stance), initialPose * o.local, noise);
      else
        graph.emplace_shared<ContactPoseFactor>(
            rootKey(i), contactKey(o.stance), noise, o.local);
    }

  // Optional zero-motion priors couple every pair of selected roots, not just
  // disconnected components. They are explicit modeling assumptions; there is
  // no measured root motion or IMU integration behind these factors.
  if (options.addBetweenFactors) {
    sigmas.head<3>().setConstant(options.betweenRotationSigma);
    sigmas.tail<3>().setConstant(options.betweenTranslationSigma);
    const auto motion = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    for (size_t i = 1; i < observations.size(); ++i) {
      if (i == 1)
        graph.emplace_shared<gtsam::PriorFactor<Pose3>>(rootKey(i), initialPose,
                                                        motion);
      else
        graph.emplace_shared<gtsam::BetweenFactor<Pose3>>(
            rootKey(i - 1), rootKey(i), Pose3(), motion);
    }
  }

  // Reuse the point-contact initializer's attitude factor. The first root is
  // fixed exactly, so its constant upright cost is omitted. Heading and all
  // translation components remain free; this is a soft roll/pitch assumption.
  if (options.addUprightFactors)
    for (size_t i = 1; i < observations.size(); ++i)
      graph.emplace_shared<contact_init::UprightFactor>(rootKey(i),
                                                        options.uprightSigma);

  // Ground objectives act once per landmark. Unless a height was supplied, use
  // the mean seeded ground-contact height at the anchored first selected slice.
  // Height and sole-normal alignment leave world XY and yaw unconstrained.
  if (options.addGroundFactors) {
    double height = 0;
    size_t count = 0;
    for (const auto& o : observations.front())
      if (definitions.at(o.stance).isGround) {
        height += contacts.at(o.stance).z();
        ++count;
      }
    height = options.groundHeight.value_or(count ? height / count : 0.0);
    require(std::isfinite(height), "invalid ground height");
    for (const auto& [stance, pose] : contacts)
      if (definitions.at(stance).isGround) {
        graph.emplace_shared<ContactHeightFactor>(
            contactKey(stance),
            gtsam::noiseModel::Isotropic::Sigma(1, options.groundSigma),
            gtsam::Point3::Zero(), gtsam::Vector3(0, 0, -1), height);
        if (options.addNormalFactors)
          graph.emplace_shared<ContactNormalFactor>(
              contactKey(stance), Pose3(), gtsam::Vector3::UnitZ(),
              gtsam::noiseModel::Isotropic::Sigma(3, options.normalSigma));
      }
  }

  const auto built = Clock::now();

  // Capture both the weighted objective (twice GTSAM's error) and unweighted
  // physical discrepancies. Translation RMS is Euclidean world displacement;
  // rotation RMS is the norm of the relative rotation logarithm, in radians.
  // Include the fixed root in snapshots, although it is absent from LM Values.
  auto capture = [&](const Values& free, size_t iteration) {
    pose_contact_init::Snapshot snapshot;
    snapshot.iteration = iteration;
    snapshot.objective = 2 * graph.error(free);
    for (size_t i = 0; i < observations.size(); ++i)
      snapshot.poses.push_back(i ? free.at<Pose3>(rootKey(i)) : initialPose);

    double translation = 0, rotation = 0;
    size_t n = 0;
    for (const auto& [stance, pose] : contacts)
      snapshot.contactPoses.push_back(free.at<Pose3>(contactKey(stance)));

    for (size_t i = 0; i < observations.size(); ++i)
      for (const auto& o : observations[i]) {
        const auto predicted = snapshot.poses[i] * o.local;
        const auto actual = free.at<Pose3>(contactKey(o.stance));
        translation +=
            (predicted.translation() - actual.translation()).squaredNorm();
        rotation +=
            gtsam::Rot3::Logmap(predicted.rotation().between(actual.rotation()))
                .squaredNorm();
        ++n;
      }

    snapshot.rms = std::sqrt(translation / n);
    result.rotationRms = std::sqrt(rotation / n);
    return snapshot;
  };

  auto snapshot = capture(seed, 0);
  result.objectiveInitial = snapshot.objective;
  if (options.recordHistory) result.history.push_back(snapshot);

  // Record each completed outer LM iteration. Internal damping trials are not
  // separate snapshots. Small objective improvement marks numerical
  // convergence; reaching maxIterations leaves converged=false, even if the fit
  // is finite.
  gtsam::LevenbergMarquardtParams params;
  params.setMaxIterations(options.maxIterations);
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, seed, params);
  for (size_t i = 0; i < options.maxIterations; ++i) {
    const double before = optimizer.error();
    optimizer.iterate();
    ++result.iterations;
    require(
        std::isfinite(optimizer.error()) && optimizer.error() <= before + 1e-8,
        "invalid LM cost");
    snapshot = capture(optimizer.values(), result.iterations);
    if (options.recordHistory) result.history.push_back(snapshot);

    // LM can return unchanged values after rejecting all trial steps, including
    // when its internal relative-error test stops the damping search. That is
    // not evidence of convergence unless the gradient is also negligible.
    if (optimizer.error() >= before) {
      const double gradientNorm =
          graph.linearize(optimizer.values())->gradientAtZero().norm();
      result.converged = gradientNorm <= 1e-6;
      result.stalled = !result.converged;
      break;
    }

    if (before - optimizer.error() <= 1e-12 ||
        before - optimizer.error() <= 1e-8 * before) {
      result.converged = true;
      break;
    }
  }

  // Diagnostics expose the sparse graph and its solution. Reinsert the exact
  // root gauge for inspection; its extra key is unused by the stored graph.
  result.graph = graph;
  result.values = optimizer.values();
  result.values.insert(rootKey(0), initialPose);
  result.poses = snapshot.poses;
  result.contactPoses = snapshot.contactPoses;
  result.residualRms = snapshot.rms;
  result.objectiveFinal = snapshot.objective;

  // These timings include sparse construction, optimization, and diagnostics,
  // but exclude the output FK expansion/interpolation below.
  const auto done = Clock::now();
  const auto ms = [](auto a, auto b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
  };
  result.timings.graphBuildMs = ms(begin, built);
  result.timings.optimizationMs = ms(built, done);
  result.timings.totalMs = ms(begin, done);

  // Convert solved root poses back into articulated configurations using the
  // measured joints. Sparse mode returns ONLY the selected source slices and
  // takes their optimized poses directly; it never calls interpolation.
  Values output;
  size_t interval = 0, sparseIndex = 0;
  for (const auto& [k, active] : frames) {
    if (!options.interpolateOutput && !selected.count(k)) continue;
    Pose3 pose;
    if (!options.interpolateOutput) {
      pose = result.poses.at(sparseIndex++);
    } else {
      // Legacy dense output interpolates between selected roots and holds the
      // nearest solved pose before/after the supported span. These added poses
      // are output configurations, not variables in the initialization graph.
      while (interval + 1 < result.poses.size() &&
             result.selectedSlices[interval + 1].k < k)
        ++interval;
      pose = result.poses[interval];
      if (k > result.selectedSlices.front().k &&
          interval + 1 < result.poses.size()) {
        const size_t first = result.selectedSlices[interval].k,
                     last = result.selectedSlices[interval + 1].k;
        pose = gtsam::interpolate<Pose3>(pose, result.poses[interval + 1],
                                         double(k - first) / (last - first));
      }
    }

    auto known = measuredSlice(robot, measuredJoints, k);
    InsertPose(&known, root->id(), k, pose);
    const auto fk = robot.forwardKinematics(known, k, rootLinkName);
    auto slice = ZeroValues(robot, k, 0.0);
    slice.update(fk);
    output.insert(slice);
  }

  if (diagnostics) *diagnostics = std::move(result);
  return output;
}

gtsam::Values Initializer::InitializeSolutionPoseIK(
    const Robot& robot, const std::string& rootLinkName,
    const std::vector<gtsam::Pose3>& rootGuesses,
    const gtsam::Values& measuredJoints, const Trajectory& trajectory,
    size_t maxIterations,
    const InitializationIterationCallback& iterationCallback) const {
  const auto frames = contactPoseFrames(trajectory);
  require(rootGuesses.size() == frames.size() && maxIterations > 0,
          "IK root guesses must match slices");

  // IK solves a separate articulated configuration at every source slice.
  // Stance IDs come from the same full-phase association as the joint contact
  // initializer, but each stance's world target is frozen at first observation.
  KinematicsParameters parameters;
  parameters.prior_q_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, 0.03);
  const Kinematics kinematics(parameters);
  std::map<size_t, gtsam::Pose3> targets;
  gtsam::Values result;
  size_t ordinal = 0;

  for (const auto& [k, contacts] : frames) {
    // Guesses are in traversal order; variable keys retain the original slice
    // k.
    auto known = measuredSlice(robot, measuredJoints, k);
    InsertPose(&known, robot.link(rootLinkName)->id(), k,
               rootGuesses.at(ordinal++));
    auto seed = robot.forwardKinematics(known, k, rootLinkName);

    auto graph = kinematics.graph(Slice(k), robot);
    graph.add(kinematics.jointAngleObjectives(Slice(k), robot, measuredJoints));
    graph.addPrior(PoseKey(robot.link(rootLinkName)->id(), k),
                   rootGuesses.at(ordinal - 1),
                   gtsam::noiseModel::Isotropic::Sigma(6, 0.001));

    for (const auto& contact : contacts) {
      // emplace preserves an existing stance target. Convert world_T_contact
      // back to world_T_linkCOM for the link-pose prior used by this IK graph.
      targets.emplace(contact.stance, contact.contact.predict(seed, k));
      const auto goal =
          targets.at(contact.stance) * contact.contact.comTcontact.inverse();
      gtsam::Vector6 sigmas;
      sigmas << .05, .05, .05, .01, .01, .01;
      graph.addPrior(PoseKey(contact.contact.link->id(), k), goal,
                     gtsam::noiseModel::Diagonal::Sigmas(sigmas));
    }

    std::vector<PoseOnLink> active;
    for (const auto& c : contacts) active.push_back(c.contact);
    graph.add(kinematics.contactPoseGroundObjectives(Slice(k), active,
                                                     gtsam::Vector3(0, 0, -1)));

    // FK may contain velocities, while this IK graph only contains q and poses.
    seed = seed.extract(graph.keyVector());

    // Callback iteration zero is the seed; subsequent callbacks are completed
    // outer LM iterations for this slice, not a whole-trajectory optimization.
    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(maxIterations);
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, seed, params);
    if (iterationCallback)
      iterationCallback(k, 0, optimizer.error(), optimizer.values());
    for (size_t i = 0; i < maxIterations; ++i) {
      double before = optimizer.error();
      optimizer.iterate();
      if (iterationCallback)
        iterationCallback(k, i + 1, optimizer.error(), optimizer.values());
      if (before - optimizer.error() < 1e-10) break;
    }

    result.insert(optimizer.values());
  }

  return result;
}

}  // namespace gtdynamics

/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file ContactTrajectoryInitializer.cpp
 * Adapt measured GTDynamics joint trajectories to the sparse contact core.
 *
 * Read the trajectory contact phases, freeze root-relative FK at contact
 * boundaries and supplemental frames, solve root poses and stance landmarks,
 * then interpolate root seeds and expand all requested link states through FK.
 * Dense expansion supplies initialization values, not new solver observations.
 */

#include <gtdynamics/utils/Initializer.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtdynamics/utils/values.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

namespace gtdynamics {
namespace {

void Require(bool valid, const std::string& message) {
  if (!valid) {
    throw std::invalid_argument("InitializeSolutionContacts: " + message);
  }
}

// Copy only the requested slice: unrelated poses in measured_values must not
// leak a reference trajectory into the initializer or constrain FK.
gtsam::Values MeasuredSlice(const Robot& robot,
                            const gtsam::Values& measured_values,
                            const Slice& slice) {
  gtsam::Values measurements;
  for (const auto& joint : robot.joints()) {
    const auto q = JointAngleKey(joint->id(), slice.k);
    Require(measured_values.exists(q), "missing joint angle for " +
                                           joint->name() + " at sample " +
                                           std::to_string(slice.k));
    const double angle = measured_values.at<double>(q);
    Require(std::isfinite(angle), "joint angles must be finite");
    measurements.insert(q, angle);
    const auto v = JointVelKey(joint->id(), slice.k);
    const double velocity =
        measured_values.exists(v) ? measured_values.at<double>(v) : 0.0;
    Require(std::isfinite(velocity), "joint velocities must be finite");
    measurements.insert(v, velocity);
  }
  return measurements;
}

// Internal landmark bookkeeping, derived from phases rather than a second
// public schedule. PointOnLink equality is also used by WalkCycle.
struct TrackedContact {
  PointOnLink point;
  std::string stance;
  size_t index;
};

using ContactFrames = std::map<size_t, std::vector<TrackedContact>>;

ContactFrames TrackContacts(const Robot& robot, const Trajectory& trajectory) {
  const auto& phases = trajectory.phases();
  Require(!phases.empty(), "trajectory must contain phases");
  ContactFrames frames;
  std::vector<TrackedContact> previous;
  size_t next_index = 0;
  for (size_t i = 0; i < phases.size(); ++i) {
    const auto& phase = phases[i];
    Require(phase.k_start <= phase.k_end, "phase interval is reversed");
    Require(phase.k_end <= static_cast<size_t>(std::numeric_limits<int>::max()),
            "sample indices exceed GTDynamics key range");
    if (i) {
      const size_t end = phases[i - 1].k_end;
      Require((phase.k_start == end || phase.k_start == end + 1) &&
                  phase.k_end > end,
              "phases must be ordered and contiguous with at most one shared "
              "endpoint");
    }
    const auto spec = phase.constraintSpec();
    Require(static_cast<bool>(spec), "phases require a ConstraintSpec");
    std::vector<TrackedContact> current;
    for (const auto& cp : spec->contactPoints()) {
      Require(static_cast<bool>(cp.link), "contact point has no link");
      Require(robot.link(cp.link->name()) == cp.link,
              "contact point link belongs to another robot");
      Require(cp.point.allFinite(), "contact point coordinates must be finite");
      Require(std::none_of(current.begin(), current.end(),
                           [&](const auto& c) { return c.point == cp; }),
              "duplicate contact point in phase");
      const auto same =
          std::find_if(previous.begin(), previous.end(),
                       [&](const auto& c) { return c.point == cp; });
      if (same != previous.end()) {
        current.push_back(*same);
      } else {
        // Group a link's material points for the core's patch seed. New points
        // get fresh indices even if other points on that link remain planted.
        const auto link = std::find_if(
            previous.begin(), previous.end(),
            [&](const auto& c) { return c.point.link == cp.link; });
        const std::string stance =
            link != previous.end() ? link->stance
                                   : cp.link->name() + ":" + std::to_string(i);
        current.push_back({cp, stance, next_index++});
      }
    }
    for (size_t k = phase.k_start; k <= phase.k_end; ++k) {
      auto& contacts = frames[k];
      for (const auto& contact : current) {
        if (std::none_of(contacts.begin(), contacts.end(), [&](const auto& c) {
              return c.point == contact.point;
            }))
          contacts.push_back(contact);
      }
    }
    previous = std::move(current);
  }
  return frames;
}

}  // namespace

gtsam::Values Initializer::InitializeSolutionContacts(
    const Robot& robot, const std::string& root_link_name,
    const gtsam::Pose3& wTroot_i, const gtsam::Values& measured_joints,
    const Trajectory& trajectory, double dt,
    const contact_init::Options& options,
    contact_init::Result* diagnostics) const {
  Require(std::isfinite(dt) && dt > 0, "dt must be finite and positive");
  const auto root = robot.link(root_link_name);
  Require(static_cast<bool>(root), "unknown root link " + root_link_name);
  Require(!root->isFixed(), "root link must be floating");
  const auto frames = TrackContacts(robot, trajectory);

  std::set<size_t> event_indices;
  for (const auto& phase : trajectory.phases()) {
    // Retain both sides of each phase transition. Flight has no observations;
    // its dense poses will be interpolated after solving supported frames.
    for (size_t k : {phase.k_start, phase.k_end})
      if (!frames.at(k).empty()) event_indices.insert(k);
    if (phase.k_start == phase.k_end) continue;
    const size_t stride = static_cast<size_t>(
        std::max(1.0, std::min(static_cast<double>(phase.k_end - phase.k_start),
                               std::floor((1.0 + 1e-8) / dt))));
    for (size_t k = phase.k_start; phase.k_end - k > stride;) {
      k += stride;
      if (!frames.at(k).empty()) event_indices.insert(k);
    }
  }
  Require(!event_indices.empty(),
          "trajectory must have at least one active contact");

  std::vector<contact_init::ContactMeasurements> measurements;
  for (size_t sample : event_indices) {
    const Slice slice(sample);
    auto local_known = MeasuredSlice(robot, measured_joints, slice);
    // An identity root COM frame makes every FK point a fixed root measurement.
    InsertPose(&local_known, root->id(), sample, gtsam::Pose3());
    const auto local_fk =
        robot.forwardKinematics(local_known, sample, root_link_name);
    contact_init::ContactMeasurements measurement{slice, sample * dt, {}};
    for (const auto& contact : frames.at(sample)) {
      measurement.observations.push_back(
          {contact.stance, contact.point.link->name(), contact.index,
           contact.point.predict(local_fk, sample)});
    }
    measurements.push_back(std::move(measurement));
  }

  auto solver_options = options;
  // The caller's first root COM pose is the exact gauge. Sparse root keys use
  // original sample indices so returned Values match normal GTDynamics keys.
  solver_options.anchorPose = wTroot_i;
  solver_options.poseKey = [root_id = root->id(), &measurements](size_t i) {
    return PoseKey(root_id, measurements.at(i).slice.k);
  };
  auto result = contact_init::ContactInitializer().Initialize(measurements,
                                                              solver_options);
  // Weak motion can explicitly join components; independent gauges alone cannot
  // define the single world trajectory required for dense interpolation.
  Require(result.graph.components.size() == 1,
          "contact graph is disconnected; independent stance components cannot "
          "initialize one trajectory without additional measurements");

  gtsam::Values values;
  size_t interval = 0;
  // Preserve measured joint angles/velocities while interpolating only root
  // pose. ZeroValues seeds the remaining dynamics quantities, including root
  // twists.
  for (const auto& [sample, active_contacts] : frames) {
    while (interval + 1 < measurements.size() &&
           measurements[interval + 1].slice.k < sample)
      ++interval;
    auto pose = result.poses[interval];
    if (sample > measurements.front().slice.k &&
        interval + 1 < measurements.size()) {
      const auto start = measurements[interval].slice.k;
      const auto end = measurements[interval + 1].slice.k;
      const double fraction =
          static_cast<double>(sample - start) / (end - start);
      pose = gtsam::interpolate<gtsam::Pose3>(pose, result.poses[interval + 1],
                                              fraction);
    }
    auto known = MeasuredSlice(robot, measured_joints, Slice(sample));
    InsertPose(&known, root->id(), sample, pose);
    auto fk = robot.forwardKinematics(known, sample, root_link_name);
    PointOnLinks points;
    for (const auto& contact : active_contacts) points.push_back(contact.point);
    // ZeroValues already seeds one aggregate wrench per link for sole patches.
    auto slice = ZeroValues(robot, sample, 0.0, points);
    slice.update(fk);
    values.insert(slice);
  }
  if (diagnostics) *diagnostics = std::move(result);
  return values;
}
}  // namespace gtdynamics

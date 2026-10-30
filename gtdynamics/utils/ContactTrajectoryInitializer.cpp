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
 * Validate the explicit stance schedule, freeze root-relative FK at contact
 * boundaries and supplemental frames, solve root poses and stance landmarks,
 * then interpolate root seeds and expand all requested link states through FK.
 * Dense expansion supplies initialization values, not new solver observations.
 */

#include <gtdynamics/utils/Initializer.h>
#include <gtdynamics/utils/values.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
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
                            size_t sample) {
  gtsam::Values slice;
  for (const auto& joint : robot.joints()) {
    const auto q = JointAngleKey(joint->id(), sample);
    Require(measured_values.exists(q), "missing joint angle for " +
                                           joint->name() + " at sample " +
                                           std::to_string(sample));
    const double angle = measured_values.at<double>(q);
    Require(std::isfinite(angle), "joint angles must be finite");
    slice.insert(q, angle);
    const auto v = JointVelKey(joint->id(), sample);
    const double velocity =
        measured_values.exists(v) ? measured_values.at<double>(v) : 0.0;
    Require(std::isfinite(velocity), "joint velocities must be finite");
    slice.insert(v, velocity);
  }
  return slice;
}

}  // namespace

gtsam::Values Initializer::InitializeSolutionContacts(
    const Robot& robot, const std::string& root_link_name,
    const gtsam::Pose3& wTroot_i, const gtsam::Values& measured_joints,
    const std::vector<size_t>& sample_indices,
    const std::vector<ContactEpisode>& contacts, double dt,
    const contact_init::Options& options,
    contact_init::Result* diagnostics) const {
  Require(!sample_indices.empty(), "sample_indices must not be empty");
  Require(std::isfinite(dt) && dt > 0, "dt must be finite and positive");
  Require(
      std::adjacent_find(sample_indices.begin(), sample_indices.end(),
                         std::greater_equal<size_t>()) == sample_indices.end(),
      "sample_indices must be strictly increasing");
  Require(sample_indices.back() <=
              static_cast<size_t>(std::numeric_limits<int>::max()),
          "sample indices exceed GTDynamics key range");
  const auto root = robot.link(root_link_name);
  Require(static_cast<bool>(root), "unknown root link " + root_link_name);
  Require(!root->isFixed(), "root link must be floating");
  Require(!contacts.empty(), "at least one contact episode is required");

  std::set<std::string> episode_ids;
  std::set<size_t> event_indices{sample_indices.front(), sample_indices.back()};
  for (const auto& episode : contacts) {
    Require(!episode.id.empty() && episode_ids.insert(episode.id).second,
            "each contact episode needs a unique nonempty touchdown ID");
    Require(episode.firstSample <= episode.lastSample,
            "contact episode interval is reversed");
    Require(std::binary_search(sample_indices.begin(), sample_indices.end(),
                               episode.firstSample) &&
                std::binary_search(sample_indices.begin(), sample_indices.end(),
                                   episode.lastSample),
            "contact episode endpoints must occur in sample_indices");
    Require(!episode.points.empty(), "contact episodes need points on links");
    for (const auto& cp : episode.points) {
      Require(static_cast<bool>(cp.link), "contact point has no link");
      const auto owned_link = robot.link(cp.link->name());
      Require(owned_link == cp.link,
              "contact point link belongs to another robot");
      Require(cp.point.allFinite(), "contact point coordinates must be finite");
    }
    event_indices.insert(episode.firstSample);
    event_indices.insert(episode.lastSample);
  }

  // Keep every contact boundary, supplementing long stances with measured
  // configurations so interpolation never spans more than one second.
  const std::vector<size_t> boundaries(event_indices.begin(),
                                       event_indices.end());
  size_t previous = boundaries.front();
  for (const size_t boundary : boundaries) {
    while ((boundary - previous) * dt > 1.0 + 1e-8) {
      const auto first = std::upper_bound(sample_indices.begin(),
                                          sample_indices.end(), previous);
      const auto end =
          std::upper_bound(first, sample_indices.end(), 1.0 + 1e-8,
                           [previous, dt](double seconds, size_t sample) {
                             return seconds < (sample - previous) * dt;
                           });
      Require(
          end != first,
          "requested samples are too sparse for a one-second frame interval");
      previous = *std::prev(end);
      event_indices.insert(previous);
    }
    previous = boundary;
  }

  std::vector<contact_init::EventFrame> frames;
  for (size_t sample : event_indices) {
    auto local_known = MeasuredSlice(robot, measured_joints, sample);
    // An identity root COM frame makes every FK point a fixed root measurement.
    InsertPose(&local_known, root->id(), sample, gtsam::Pose3());
    const auto local_fk =
        robot.forwardKinematics(local_known, sample, root_link_name);
    contact_init::EventFrame frame;
    frame.sampleIndex = sample;
    frame.time = sample * dt;
    for (const auto& episode : contacts) {
      if (sample < episode.firstSample || sample > episode.lastSample) continue;
      for (size_t j = 0; j < episode.points.size(); ++j) {
        const auto& cp = episode.points[j];
        frame.observations.push_back(
            {episode.id, cp.link->name(), j, cp.predict(local_fk, sample)});
      }
    }
    Require(
        !frame.observations.empty(),
        "each selected frame and horizon endpoint must have an active contact");
    frames.push_back(std::move(frame));
  }

  auto solver_options = options;
  // The caller's first root COM pose is the exact gauge. Sparse root keys use
  // original sample indices so returned Values match normal GTDynamics keys.
  solver_options.anchorPose = wTroot_i;
  solver_options.poseKey = [root_id = root->id(), &frames](size_t i) {
    return PoseKey(root_id, frames.at(i).sampleIndex);
  };
  auto result =
      contact_init::ContactInitializer().Initialize(frames, solver_options);
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
  for (size_t sample : sample_indices) {
    while (interval + 1 < frames.size() &&
           frames[interval + 1].sampleIndex < sample)
      ++interval;
    auto pose = result.poses[interval];
    if (interval + 1 < frames.size()) {
      const auto start = frames[interval].sampleIndex;
      const auto end = frames[interval + 1].sampleIndex;
      const double fraction =
          static_cast<double>(sample - start) / (end - start);
      pose = gtsam::interpolate<gtsam::Pose3>(pose, result.poses[interval + 1],
                                              fraction);
    }
    auto known = MeasuredSlice(robot, measured_joints, sample);
    InsertPose(&known, root->id(), sample, pose);
    auto fk = robot.forwardKinematics(known, sample, root_link_name);
    PointOnLinks active_contacts;
    for (const auto& episode : contacts) {
      if (sample >= episode.firstSample && sample <= episode.lastSample) {
        active_contacts.insert(active_contacts.end(), episode.points.begin(),
                               episode.points.end());
      }
    }
    // ZeroValues supports one wrench per link; a sole patch may contain many
    // geometric points on that link. Deduplicate the zero-seed slots.
    PointOnLinks wrench_contacts;
    std::set<size_t> contact_link_ids;
    for (const auto& cp : active_contacts) {
      if (contact_link_ids.insert(cp.link->id()).second)
        wrench_contacts.push_back(cp);
    }
    auto slice = ZeroValues(robot, sample, 0.0, wrench_contacts);
    slice.update(fk);
    values.insert(slice);
  }
  if (diagnostics) *diagnostics = std::move(result);
  return values;
}
}  // namespace gtdynamics

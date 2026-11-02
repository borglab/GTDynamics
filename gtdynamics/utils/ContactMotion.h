/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
#pragma once

#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/geometry/Pose3.h>

#include <vector>

namespace gtdynamics {

/** Thresholds for deriving stationary-contact training targets from world
 * motion. The centered window uses past and future reference poses: this is an
 * offline, privileged labeler, not a proprioceptive contact detector. No force,
 * terrain height, or load-bearing inference is performed. A stationary raised
 * foot is indistinguishable from a stationary supported foot without additional
 * cues.
 */
struct ContactMotionParams {
  double windowSeconds = 0.2;    ///< Full centered window duration, seconds.
  double stationarySpeed = 0.1;  ///< Maximum stationary point speed, m/s.
  double movingSpeed = 0.25;     ///< Minimum definitely moving speed, m/s.
  double stationaryExcursion = 0.01;  ///< Stationary excursion bound, metres.
  double movingExcursion = 0.025;     ///< Definitely moving excursion, metres.
};

/** One target aligned with one input pose. Labels: 1 stationary, 0 moving,
 * -1 uncertain (including incomplete windows). The score is a deterministic
 * threshold margin in [0,1], not a calibrated contact probability.
 */
struct ContactMotionSample {
  int label = -1;
  double score = 0;
  bool completeWindow = false;
  double maxSpeed = 0;      ///< Largest endpoint speed among material points.
  double maxExcursion = 0;  ///< Largest distance from the central point pose.
  double angularExcursion = 0;  ///< Maximum rotation from center, radians.
  double angularSpeed =
      0;  ///< Endpoint SO(3) rotation magnitude / span, rad/s.
  std::vector<double> pointSpeeds;
  std::vector<double> pointExcursions;
};

/** Derive stationary-contact targets from world_T_foot poses and material
 * points.
 *
 * times are finite, strictly increasing seconds, aligned with worldTfoot.
 * points are finite coordinates in the same foot frame used by worldTfoot;
 * use noncollinear sole points to observe pivots about a foot origin or corner.
 * For each centered window, point speed is endpoint displacement / actual time
 * span. Excursion is the maximum distance from its position at the center
 * sample to ANY position in the window, detecting motion that returns to its
 * starting point. Both are 3D, invariant to a constant world rigid transform.
 * The largest value over ALL points drives the label; one stationary pivot
 * point cannot make the complete sole stationary. No gap filling is applied.
 *
 * Stationary requires both metrics <= their stationary bounds. Moving requires
 * either metric >= its moving bound. Intermediate margins are uncertain.
 * A complete window requires source coverage on both sides and >=2 samples.
 * Returns one sample per pose, including uncertain boundary samples. Throws
 * std::invalid_argument for mismatched/empty input, invalid times, nonfinite
 * geometry, or nonpositive/unordered thresholds. Default params use a 200 ms
 * window, 0.1/0.25 m/s speed bounds and 10/25 mm excursion bounds.
 */
std::vector<ContactMotionSample> contactMotionLabels(
    const std::vector<double>& times,
    const std::vector<gtsam::Pose3>& worldTfoot,
    const std::vector<gtsam::Point3>& points,
    const ContactMotionParams& params = {});

/** Pose stationarity thresholds in metres, seconds and radians. Contact frame
 * translation and rotation both participate; pure pivots cannot appear still.
 * Labels are privileged motion targets, not load-bearing contact measurements.
 */
struct ContactPoseMotionParams {
  double windowSeconds = 0.2;
  double stationarySpeed = 0.1;
  double movingSpeed = 0.25;
  double stationaryExcursion = 0.01;
  double movingExcursion = 0.025;
  double stationaryAngularSpeed = 0.3;
  double movingAngularSpeed = 0.8;
  double stationaryAngularExcursion = 0.03;
  double movingAngularExcursion = 0.08;
};

/** Label world contact poses using origin translation and SO(3) rotation.
 * The centered window checks endpoint speeds and maximum excursions from its
 * central pose. All four metrics must pass stationary bounds; any metric at
 * its moving bound makes the target moving. Intermediate/incomplete windows
 * are uncertain. Score is a threshold margin, not a calibrated probability.
 * maxSpeed/maxExcursion describe the contact origin. Angular quantities are
 * rad/s and radians. Times must be finite and strictly increasing.
 */
std::vector<ContactMotionSample> contactPoseMotionLabels(
    const std::vector<double>& times,
    const std::vector<gtsam::Pose3>& worldTcontact,
    const ContactPoseMotionParams& params = {});

/** Apply a walking support prior to one foot's raw pose-motion samples.
 * Both arrays must be aligned to the same times/windows and generated with
 * params. Returns raw labels (1 stationary, 0 moving, -1 uncertain), or 2 for
 * likely contact inferred from the other foot's translation. A complete window
 * on both feet is required. The other origin must meet either moving
 * translation bound; this origin must meet neither. Raw stationary labels are
 * preserved. Rotation alone cannot trigger the prior, but the supporting foot
 * may rock. Call again with arrays swapped for the other foot; never feed
 * inferred labels back into the rule. Scores and input samples remain
 * unchanged.
 *
 * This is a walking assumption, not a probability or a stationary-sole claim.
 * It cannot establish support during flight or distinguish a sliding swing foot
 * from a loaded foot. Simultaneous clear translation gives no promotion.
 * Throws std::invalid_argument for empty/mismatched arrays, invalid raw labels,
 * nonfinite/negative translation metrics, or nonpositive moving bounds.
 */
std::vector<int> walkingContactLabels(
    const std::vector<ContactMotionSample>& foot,
    const std::vector<ContactMotionSample>& otherFoot,
    const ContactPoseMotionParams& params = {});

/** Binary-foot support model for 50 Hz slices. Each foot is 0=swing or
 * 1=stance. Joint configurations index the transition as 2*left+right, ordered
 * (WW, WS, SW, SS); rows are previous and columns next configurations.
 * Defaults use independent per-foot persistence .97 and switching .03.
 * The separate within-slice coupling penalizes WW by noSupportWeight.
 * Scales are metres per incoming sample interval, not velocities. All values
 * are illustrative assumptions, not fitted probabilities.
 */
struct ContactSupportParams {
  double supportSigma = 0.003;
  double swingSigma = 0.020;
  double noSupportWeight = 0.01;
  /// Weight for WSW or SWS across three slices; 1 disables the soft penalty.
  double singleSliceWeight = 0.1;
  gtsam::Matrix4 transition =
      (gtsam::Matrix4() << .9409, .0291, .0291, .0009, .0291, .9409, .0009,
       .0291, .0291, .0009, .9409, .0291, .0009, .0291, .0291, .9409)
          .finished();
};

/** Build a discrete graph with two binary variables per slice: left key 2*k,
 * right key 2*k+1, 0=swing and 1=stance. N incoming displacements per foot
 * give N slices (N>=2). Input d[k] is ||p[k]-p[k-1]|| in metres; the caller
 * must supply one preceding pose to compute d[0]. Arrays must be aligned,
 * finite, nonnegative, sampled at the configured interval (default 20 ms).
 *
 * Each edge has a four-variable transition factor and TWO binary displacement
 * factors, one per foot. For foot states a,b at slices k,k+1, the latter is
 * h(d[k];sigma[a])^w[k] * h(d[k+1];sigma[b])^w[k+1], where h is half-normal,
 * sigma[0]=swingSigma, sigma[1]=supportSigma, and w=1 at sequence ends, .5
 * elsewhere. Thus WW, WS, SW, SS score broad/broad, broad/narrow,
 * narrow/broad, narrow/narrow. Overlap weights count every measurement once.
 * These initial binary potentials are separable; no extra displacement
 * correlation is assumed. The transition factor supplies temporal dependence.
 * Tables are max-rescaled, log ratios floored at -700, and d/sigma capped
 * at 1e6 before squaring. The density normalization 1/sigma matters.
 *
 * A uniform initial pair prior and N within-slice coupling factors (weight
 * noSupportWeight for WW, 1 otherwise) are supplemented by 2*(N-2)
 * duration factors. Each connects three consecutive slices of ONE foot,
 * assigning singleSliceWeight to WSW/SWS and 1 to the other six patterns.
 * This soft prior penalizes an isolated 20 ms classification at 50 Hz;
 * it imposes no hard minimum duration and does not penalize sequence ends.
 * There are 6*N-6 factors, including neutral duration factors if the weight
 * is 1. Simultaneous swing is discouraged, not forbidden. Use
 * graph.optimize(Ordering::NATURAL) for exact max-product and traceback. Pure
 * pivots are not detected. Throws std::invalid_argument for invalid arrays,
 * scales, coupling weight and single-slice weight (both must lie in (0,1]), or
 * transitions (positive row-stochastic matrix).
 */
gtsam::DiscreteFactorGraph contactSupportGraph(
    const std::vector<double>& leftDisplacements,
    const std::vector<double>& rightDisplacements,
    const ContactSupportParams& params = {});

/** Build optional hard walking constraints for N slices (N>=1).
 * Uses the contactSupportGraph binary keys 2*k (left), 2*k+1 (right),
 * with 0=swing and 1=stance. Append the returned graph to enable constraints;
 * existing measurement, duration, and transition factors are unchanged.
 * N binary factors forbid both-swing within each slice. N-1 four-variable
 * factors forbid left-only to right-only transitions and vice versa. All
 * other entries are 1, forbidden entries are EXACTLY 0 (never floored).
 * Thus support exchanges require an intervening double-stance slice, without
 * prescribing a physical duration. Returns 2*N-1 factors. Throws
 * std::invalid_argument for zero slices.
 */
gtsam::DiscreteFactorGraph contactWalkingConstraints(size_t sliceCount);

/** Build two unary support factors per sample from foot forward velocities
 * relative to the pelvis, expressed in pelvis axes (+X forward), in m/s.
 * These are joint-kinematic velocities, not world velocities. For velocity v,
 * c(v) = clamp(1/(1+exp(v/velocityScale)), minimumProbability,
 *              1-minimumProbability).
 * Each foot has weights [1-c, c] in swing/stance order.
 * Each factor is agnostic about the other foot. Zero velocity
 * is neutral; forward favors swing, backward favors support. The bounded
 * sigmoid is a walking heuristic, not a learned/calibrated sensor model;
 * backward walking or external leg motion can violate it.
 *
 * N aligned velocities give unary factors on binary keys 2*k (left) and
 * 2*k+1 (right). Append to contactSupportGraph for the same slices.
 * No initial/transition factors are added here. Throws std::invalid_argument
 * for empty/mismatched/nonfinite inputs, nonpositive/nonfinite velocityScale,
 * or minimumProbability outside the open interval (0, 0.5).
 */
gtsam::DiscreteFactorGraph contactVelocityFactors(
    const std::vector<double>& leftForwardVelocities,
    const std::vector<double>& rightForwardVelocities,
    double velocityScale = 0.1, double minimumProbability = 0.02);

}  // namespace gtdynamics

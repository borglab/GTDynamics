/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file KinodynamicInitializer.h
 * @brief Contact, GP and affine linear dynamics initialization.
 * @author GTDynamics contributors
 */
#pragma once
#include <gtdynamics/dynamics/FixedKinematicsDynamics.h>
#include <gtdynamics/kinematics/ContactInterpolationInitializer.h>
namespace gtdynamics {
/** Differentiate fitted velocities with [-1, 0, 1]/(2h), clipped to the
 * interpolation range using the actual elapsed time at boundaries. Root
 * acceleration is the ordinary derivative of body-twist components, not world
 * acceleration: physical body linear acceleration is vdot + omega cross v.
 * Joint means are rad/s^2 (or m/s^2 for prismatic joints). No inputs change.
 */
gtsam::Values gpAccelerationPriors(
    const FloatingBaseInterpolation& interpolation, size_t rootId, double time,
    size_t k, double stepSeconds = 0.02);
/** Immutable strategy configuration. Dynamics settings share the full solver's
 * gravity and acceleration-prior scales. Effort/rate costs are not constructed;
 * no dynamics optimizer iterations or manifold projections are performed.
 */
struct KinodynamicInitializationOptions {
  ContactInterpolationOptions kinematics;
  fixed_dynamics::Options dynamics;
  double accelerationStepSeconds = 0.02;
};
/** Owning pipeline results. values merges fixed GP kinematics and inferred
 * dynamics, plus WORLD root COM linear velocities for the Galilean IMU factor.
 * Biases/stance-landmark variables for a later coupled solve are not created.
 * Intermediate state maps are not retained. contact and interpolation own the
 * sparse fit and GP data; dynamics contains only linear seed diagnostics.
 * totalSeconds
 * includes every stage, including differentiation, FK and result assembly.
 */
struct KinodynamicInitializationResult {
  gtsam::Values values;
  pose_contact_init::Result contact;
  std::shared_ptr<FloatingBaseInterpolation> interpolation;
  fixed_dynamics::Diagnostics dynamics;
  double contactSeconds = 0;
  double interpolationSeconds = 0;
  double dynamicsSeconds = 0;
  double totalSeconds = 0;
};
/** Run contact initialization, GP velocity fitting, then one affine dynamics
 * seed solve. The full Dynamics app optimization remains a separate operation.
 * The later coupled kinodynamic solve optimizes effort and temporal costs.
 */
class KinodynamicInitializer {
 public:
  /** Bind the robot and copy tuning parameters. The model shares fixed
   * Link/Joint topology, which must remain unchanged. Trajectory data is
   * supplied separately to run(). */
  explicit KinodynamicInitializer(
      const Robot& robot, const KinodynamicInitializationOptions& options = {});

  /** Infer forces/torques and accelerations at every requested source slice.
   * All kinematic input/output frames follow ContactInterpolationInitializer.
   * Sole wrenches use contact axes and origin; armature is indexed by joint ID.
   * No reference trajectory, IMU signal or fitted dynamics are required as
   * inputs. Exact dynamics failure propagates to the caller; no soft fallback.
   */
  KinodynamicInitializationResult run(
      const std::string& rootLinkName, const gtsam::Pose3& initialPose,
      const gtsam::Values& measuredJoints, const Trajectory& trajectory,
      const std::vector<Slice>& slices, const std::vector<double>& times,
      double dt, const std::vector<fixed_dynamics::Contact>& contacts,
      const gtsam::Vector& armature,
      const std::vector<Slice>& selectedSlices = {}) const;

 private:
  const Robot robot_;
  const KinodynamicInitializationOptions options_;
};
}  // namespace gtdynamics

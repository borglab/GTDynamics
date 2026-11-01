/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  initialize_solution_utils.h
 * @brief Utility methods for initializing trajectory optimization solutions.
 * @author: Alejandro Escontrela, Yetong Zhang
 */

#pragma once

#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/utils/PointOnLink.h>
#include <gtdynamics/utils/ContactInitializer.h>
#include <gtsam/base/Value.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/slam/PriorFactor.h>

#include <optional>
#include <random>
#include <string>
#include <vector>

namespace gtdynamics {

class Trajectory;

/// Observe each per-slice IK seed (iteration zero) and completed LM iteration.
/// Values are valid during the callback; copy them to retain a state.
using InitializationIterationCallback =
    std::function<void(size_t, size_t, double, const gtsam::Values&)>;

class Initializer {
 private:
  const Kinematics kinematics_;

 public:
  // Default Constructor
  Initializer();

    /**
     * Add zero-mean gaussian noise to a Pose3.
     *
     * @param T Pose3 which to add noise to.
     * @param sampler Helper to sample values from a gaussian distribution.
     */
    gtsam::Pose3 AddGaussianNoiseToPose(const gtsam::Pose3& T,
                                            const gtsam::Sampler& sampler) const;

    /**
     * Linearly interpolate between initial pose and desired poses at each
     * specified, discrete timestep.
     *
     * @param wTl_i Initial pose of the link.
     * @param wTl_t Vector of desired poses.
     * @param t_i Initial time.
     * @param timesteps Times at which poses start and end.
     * @param dt The duration of a single timestep.
     */
    std::vector<gtsam::Pose3> InterpolatePoses(
        const gtsam::Pose3& wTl_i, const std::vector<gtsam::Pose3>& wTl_t,
        double t_i, const std::vector<double>& timesteps, double dt);

    /**
     * Initialize the poses and joints needed to perform trajectory optimization.
     *
     * @param robot The robot model.
     * @param wTl_i Initial pose of the link.
     * @param wTl_t Vector of poses at each desired timestep.
     * @param link_name The name of the link whose pose to initialize.
     * @param t_i The initial time of the trajectory.
     * @param timesteps A vector of desired timesteps.
     * @param dt The duration of a single timestep.
     * @param sampler Helper to sample values from a gaussian distribution.
     */
    gtsam::Values InitializePosesAndJoints(const Robot& robot,
                                        const gtsam::Pose3& wTl_i,
                                        const std::vector<gtsam::Pose3>& wTl_t,
                                        const std::string& link_name, double t_i,
                                        const std::vector<double>& timesteps,
                                        double dt, const gtsam::Sampler& sampler,
                                        std::vector<gtsam::Pose3>* wTl_dt);

    /**
     * @fn Initialize solution via linear interpolation of initial and final pose.
     *
     * @param[in] robot           A Robot object.
     * @param[in] link_name       The name of the link whose pose to interpolate.
     * @param[in] wTl_i           The initial pose of the link.
     * @param[in] wTl_f           The final pose of the link.
     * @param[in] T_s             Time at which to start interpolation.
     * @param[in] T_f             Time at which to end interpolation.
     * @param[in] dt              The duration of a single timestep.
     * @param[in] gaussian_noise  Optional gaussian noise to add to initial values.
     *      Noise drawn from a zero-mean gaussian distribution with a standard
     *      deviation of gaussian_noise.
     * @param[in] contact_points  PointOnLink objects.
     * @return Initial solution stored in gtsam::Values object.
     */
    gtsam::Values InitializeSolutionInterpolation(
        const Robot& robot, const std::string& link_name, const gtsam::Pose3& wTl_i,
        const gtsam::Pose3& wTl_f, double T_s, double T_f, double dt,
        double gaussian_noise = 0.0,
        const std::optional<PointOnLinks>& contact_points = {});

    /**
     * @fn Initialize interpolated solution for multiple phases.
     *
     * @param[in] robot           A Robot object.
     * @param[in] link_name       The name of the link whose pose to interpolate.
     * @param[in] wTl_i           The initial pose of the link.
     * @param[in] wTl_t           A vector of desired poses.
     * @param[in] ts              Times at which poses start and end.
     * @param[in] dt              The duration of a single timestep.
     * @param[in] gaussian_noise  Optional gaussian noise to add to initial values.
     *      Noise drawn from a zero-mean gaussian distribution with a standard
     *      deviation of gaussian_noise.
     * @param[in] contact_points  PointOnLink objects.
     * @return Initial solution stored in gtsam::Values object.
     */
    gtsam::Values InitializeSolutionInterpolationMultiPhase(
        const Robot& robot, const std::string& link_name, const gtsam::Pose3& wTl_i,
        const std::vector<gtsam::Pose3>& wTl_t, const std::vector<double>& ts,
        double dt, double gaussian_noise = 0.0,
        const std::optional<PointOnLinks>& contact_points = {});

    /**
     * @fn Iteratively solve for the robot kinematics with contacts.
     *
     * @param[in] robot           A Robot object.
     * @param[in] link_name       The name of the link whose pose to interpolate.
     * @param[in] wTl_i           The initial pose of the link.
     * @param[in] wTl_t           A vector of desired poses.
     * @param[in] ts              Times at which poses start and end.
     * @param[in] dt              The duration of a single timestep.
     * @param[in] gaussian_noise  Optional gaussian noise to add to initial values.
     *      Noise drawn from a zero-mean gaussian distribution with a standard
     *      deviation of gaussian_noise.
     * @param[in] contact_points  PointOnLink objects.
     * @param[in] iteration_callback Optional observer of each slice's seed and
     * completed LM iterations: (sample index, iteration, GTSAM error, values).
     * @return Initial solution stored in gtsam::Values object.
     */
    gtsam::Values InitializeSolutionInverseKinematics(
        const Robot& robot, const std::string& link_name, const gtsam::Pose3& wTl_i,
        const std::vector<gtsam::Pose3>& wTl_t, const std::vector<double>& ts,
        double dt, double gaussian_noise = 1e-8,
        const std::optional<PointOnLinks>& contact_points = {},
        const InitializationIterationCallback& iteration_callback = {});

    /** Initialize measured joints over a trajectory of contact phases.
     *
     * Each phase supplies a ConstraintSpec with contactPoints(). All contacts
     * are used, regardless of their isGround flag. Mixed ground/wall contacts
     * require a Patch or Point model without global ground priors.
     * The same point on the same link in consecutive phases shares a landmark,
     * regardless of point ordering. A point that disappears
     * and later returns gets a new landmark. Callers supply no touchdown IDs.
     *
     * Phases use original, inclusive sample indices. They must be ordered and
     * contiguous, either disjoint or sharing one endpoint. Shared endpoints use
     * the union of contacts and produce only one output state. Empty contact
     * specifications represent flight; missing specifications are invalid.
     *
     * Phase boundaries with contact are retained for the sparse solve. Long
     * supported phases are supplemented at intervals of at most one second
     * (or every sample if dt > 1). Root poses are interpolated between solved
     * frames and held constant outside the supported span. In particular,
     * wTroot_i anchors the first supported frame and any unsupported prefix.
     * These flight/interpolation values are seeds, not contact measurements.
     * At least one supported frame is required. Disconnected contact graphs
     * require an explicit motion prior (options.addBetweenFactors).
     *
     * A positive contactBuffer excludes that many slices at both ends of
     * each link's continuous stance, even when it spans multiple phases.
     * Both retained stance endpoints are selected, with the usual phase and
     * one-second supplemental samples wherever trusted contacts remain.
     * Each observation is filtered independently of which foot selected the
     * slice. Every stance must retain at least two distinct slices, otherwise
     * std::invalid_argument is thrown; margins are never reduced. Zero keeps
     * the original selection exactly, including single-slice stances.
     * The gauge is at the first retained slice; the prefix is held at wTroot_i,
     * so no root motion is inferred in the trimmed prefix. Dense outputs keep
     * the original contact schedule. Trimming may disconnect the sparse graph.
     *
     * @param robot Robot model, with COM-frame forward kinematics.
     * @param root_link_name Link whose COM pose defines the floating base.
     * @param wTroot_i Initial world COM pose, fixing the gauge as above.
     * @param measured_joints JointAngleKey(j,k) for every joint and trajectory
     * slice. Optional JointVelKey values are preserved; absent velocities are
     * zero. Existing poses in these Values are ignored.
     * @param trajectory Phases and their contact specifications.
     * @param dt Seconds per sample index, finite and positive.
     * @param options Sparse contact solver settings; anchor/pose keys are set
     * internally to wTroot_i and PoseKey(root_link_id,k).
     * @param diagnostics Optional sparse graph, solution, and core timings.
     * diagnostics->slices() returns the exact source slices used by the solve,
     * allowing downstream estimation at those same, irregularly spaced times.
     * @param contactBuffer Number of source slices excluded at each stance
     * boundary; defaults to zero. The time margin is contactBuffer * dt.
     *
     * FK expands all trajectory slices into ordinary GTDynamics Values,
     * preserving measured joints and zero-seeding remaining dynamics variables.
     * Root twists are zero seeds. Invalid phases, contact geometry, or missing
     * or nonfinite joint measurements throw std::invalid_argument.
     */
    gtsam::Values InitializeSolutionContacts(
        const Robot& robot, const std::string& root_link_name,
        const gtsam::Pose3& wTroot_i, const gtsam::Values& measured_joints,
        const Trajectory& trajectory, double dt = 1.0,
        const contact_init::Options& options = {},
        contact_init::Result* diagnostics = nullptr,
        size_t contactBuffer = 0) const;

    /**
     * @fn Initialize solution for multi-phase trajectory to nominal pose.
     *
     * @param[in] robots                The Robot object to use for each phase.
     * @param[in] phase_steps           Number of steps for each phase.
     * @param[in] transition_graph_init Initial values for transition graphs.
     * @param[in] dt_i                  Initial phase duration,
     * @param[in] gaussian_noise  Optional gaussian noise to add to initial values.
     * @param[in] phase_contact_points  Contact points at each phase.
     */
    gtsam::Values MultiPhaseZeroValuesTrajectory(
        const Robot& robot, const std::vector<int>& phase_steps,
        std::vector<gtsam::Values> transition_graph_init, double dt_i = 1. / 240,
        const double gaussian_noise = 1e-8,
        const std::optional<std::vector<PointOnLinks>>& phase_contact_points =
            {}) const;

    /**
     * @fn Multi-phase initialize solution inverse kinematics.
     *
     * @param[in] robots                A Robot object for each phase.
     * @param[in] link_name             The name of the link whose pose to
     * interpolate.
     * @param[in] phase_steps           Number of steps for each phase.
     * @param[in] wTl_i                 The initial pose of the link.
     * @param[in] wTl_t                 A vector of desired poses.
     * @param[in] ts                    Discretized times at which poses start and
     * end.
     * @param[in] transition_graph_init Initial values for transition graphs.
     * @param[in] dt_i                  Initial value for phase duration.
     * @param[in] gaussian_noise  Optional gaussian noise to add to initial values.
     *      Noise drawn from a zero-mean gaussian distribution with a standard
     *      deviation of gaussian_noise.
     * @param[in] phase_contact_points  Contact points at each phase.
     * @return Initial solution stored in gtsam::Values object.
     */
    gtsam::Values MultiPhaseInverseKinematicsTrajectory(
        const Robot& robot, const std::string& link_name,
        const std::vector<int>& phase_steps, const gtsam::Pose3& wTl_i,
        const std::vector<gtsam::Pose3>& wTl_t, const std::vector<double>& ts,
        std::vector<gtsam::Values> transition_graph_init, double dt_i = 1. / 240,
        double gaussian_noise = 1e-8,
        const std::optional<std::vector<PointOnLinks>>& phase_contact_points =
            {});

    /**
     * @fn Return zero values for all variables for initial value of optimization.
     *
     * @param[in] robot          A Robot object.
     * @param[in] t              Timestep to return zero initial values for.
     * @param[in] gaussian_noise  Optional gaussian noise to add to initial values.
     *      Noise drawn from a zero-mean gaussian distribution with a standard
     *      deviation of gaussian_noise.
     * @param[in] contact_points Contact points for timestep t.
     */
    virtual gtsam::Values ZeroValues(
        const Robot& robot, const int t, double gaussian_noise = 0.0,
        const std::optional<PointOnLinks>& contact_points = {}) const;

    /**
     * @fn Return zero values of the trajectory for initial value of optimization.
     *
     * @param[in] robot          A Robot object.
     * @param[in] num_steps      Total number of time steps.
     * @param[in] num_phases     Number of phases, -1 if not using.
     * @param[in] gaussian_noise  Optional gaussian noise to add to initial values.
     *      Noise drawn from a zero-mean gaussian distribution with a standard
     *      deviation of gaussian_noise.
     * @param[in] contact_points Contact points along the trajectory.
     * @return Initial solution stored in a gtsam::Values object.
     */
    gtsam::Values ZeroValuesTrajectory(
        const Robot& robot, const int num_steps, const int num_phases = -1,
        double gaussian_noise = 0.0,
        const std::optional<PointOnLinks>& contact_points = {});

};

}  // namespace gtdynamics

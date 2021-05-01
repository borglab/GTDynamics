/**
 * @file  testJumpingRobot.cpp
 * @brief test trajectory optimization for jumping robot
 * @Author: Yetong Zhang
 */

#pragma once

// #include <gtsam/geometry/Point3.h>
// #include <gtsam/inference/LabeledSymbol.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <iostream>
#include <fstream>

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/jumpingrobot/factors/PneumaticActuator.h"

using gtsam::NonlinearFactorGraph, gtsam::Values;
using gtsam::Vector, gtsam::Vector3, gtsam::Vector6;

namespace gtdynamics
{

void merge(Values &values, const Values &new_values)
{
    for (gtsam::Key &key : new_values.keys())
    {
        if (!values.exists(key))
        {
            values.insert(key, new_values.at(key));
        }
    }
}

class JumpingRobot
{
public:
    enum Phase
    {
        Ground,
        Air,
        Left,
        Right
    };

    struct Params {
        double angle_offset;
        double tank_volume;
        double Rs;
        double T;
    };

public:
    JointValueMap init_angles_, init_vels_, init_masses_;
    Params params_;
    gtsam::Vector3 gravity_;
    gtsam::Vector3 planar_axis_;
    Robot robot_ground_;
    Robot robot_air_;
    Robot robot_left_;
    Robot robot_right_;
    std::vector<PneumaticActuator> actuators_;
    DynamicsGraph graph_builder_;
    int left_i_, right_i_, body_i_, left_j_, right_j_;
    const Vector3 p_end_left_ = (Vector(3) << 0, 0, 0.275).finished();
    const Vector3 p_end_right_ = (Vector(3) << 0, 0, -0.275).finished();

    /** !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! constructor related !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
    /** construct the actuators */
    std::vector<PneumaticActuator> constructActuators(const JointValueMap &rest_angles);

    /** initialize robots of 4 different phases */
    void initializeRobots();

    /**
     * Construct jumping robot
     * Keyword arguments:
            rest_angles        -- angles at rest for each joint
            init_angles        -- initial joint angles
            int_vels           -- initial joint velocities
    */
    JumpingRobot(const JointValueMap &rest_angles,
                 const JointValueMap &init_angles,
                 const JointValueMap &init_vels,
                 const JointValueMap &init_masses,
                 const Params& params);

    /** !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! basic utility !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
    static bool leftOnGround(const Phase phase)
    {
        return phase == Phase::Left || phase == Phase::Ground;
    }

    static bool rightOnGround(const Phase phase)
    {
        return phase == Phase::Right || phase == Phase::Ground;
    }

    static Phase getPhase(bool left_on_ground, bool right_on_ground)
    {
        if (left_on_ground && right_on_ground)
        {
            return Phase::Ground;
        }
        else if (left_on_ground)
        {
            return Phase::Left;
        }
        else if (right_on_ground)
        {
            return Phase::Right;
        }
        else
        {
            return Phase::Air;
        }
    }

    static std::vector<Phase> getPhases(const std::vector<Phase> phase_seq, const std::vector<int> phase_steps)
    {
        std::vector<Phase> phases;
        phases.emplace_back(phase_seq[0]);
        for (size_t phase_idx = 0; phase_idx < phase_seq.size(); phase_idx++)
        {
            for (int step = 0; step < phase_steps[phase_idx]; step++)
            {
                phases.emplace_back(phase_seq[phase_idx]);
            }
        }
        return phases;
    }

    /** !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! simulation related !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
    /**
     * check phase transition, and return phase for next step
     * Keyword arguments:
            current_phase      -- current phase
            t                  -- current time step
            values             -- values for the current step
    */
    Phase phaseTransition(const Phase current_phase,
                          const int t,
                          const Values &values) const;

    /**
     * solve the factor graph of actuators for a time step, return values
     * Keyword arguments:
            t                  -- current time step
            dt                 -- duration of tiem step
            qs                 -- joint angles
            vs                 -- joint velocities
            ms                 -- mass of air in muscle
            init_pressures     -- initial pressures for actuators
            start_times        -- time to open the valve for actuators
    */
    Values runActuators(const int t, const double dt,
                        const JointValueMap &qs,
                        const JointValueMap &vs,
                        const JointValueMap &ms,
                        const double source_mass,
                        const Vector &open_times,
                        const Vector &close_times,
                        const Values& previous_values) const;

    /**
     * extract torques at all joints from actuator results
     * Keyword arguments:
            phase              -- phase of current step
            t                  -- current time step
            values             -- results of actuator factor graph
    */
    JointValueMap extractTorques(const Phase phase,
                                      const int t,
                                      const Values &values) const;

    JointValueMap extractMdots(const Phase phase,
                                      const int t,
                                      const Values &values) const;

    // /**
    //  * Simulate for a sequence of steps, return the values of all variables
    //  * Keyword arguments:
    //         num_steps           -- number of time steps
    //         dt                  -- duration for each time step
    //         init_pressures      -- initial pressure for actuators
    //         start_times         -- start time of each actuator
    // */
    // Values simulate(const int num_steps, const double dt, const Vector &init_pressures, const Vector &start_times);

    /** !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! linear interpolate !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
    /**
     * get the values at a previous time step, with keys of the new time step
     * Keyword arguments:
            phase              -- phase of the specified step
            t_in               -- corresponding step in input trajectory
            t_out              -- step in output trajectory
            values_in          -- input trajectory values
    */
    Values valuesFromPrev(const Phase phase,
                          const int t_in, const int t_out,
                          const Values &values_in,
                          const bool specify_torque = false) const;

    /**
     * interpolate the values at specified time step
     * Keyword arguments:
            phase              -- phase of the specified step
            t_in               -- corresponding step in input trajectory
            t_out              -- step in output trajectory
            values_in          -- input trajectory values
    */
    Values getInterpolateValues(const Phase phase,
                                const double t_in, const int t_out,
                                const Values &values_in,
                                const bool specify_torque = false) const;

    /**
     * Linear interpolate the values of a trajectory
     * Keyword arguments:
            dt_in               -- duration for each time step
            phases_es_in        -- phase of each time step
            values_in           -- number of time steps
            phases_out          -- output phase sequence
            phase_steps_out     -- output number of steps in each phase
    */
    Values linearInterpolation(const double dt_in,
                               const std::vector<Phase> &phases_es_in,
                               const gtsam::Values &values_in,
                               const std::vector<Phase> &phases_out,
                               const std::vector<int> &phase_steps_out,
                               const bool specify_torque = false) const;

    /** !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! build factors !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

    /** return priors to specify the initial state */
    NonlinearFactorGraph initialStatePriors() const;

    /** return priors for control variables (init pressure and start time) */
    NonlinearFactorGraph controlPriors(const double init_tank_pressure, const Vector &open_times, const Vector &close_times) const;

    /**
     * construct the factor graph for actuators of all time steps
     * Keyword arguments:
            num_steps          -- number of time steps
    */
    NonlinearFactorGraph actuatorGraphs(const int num_steps) const;

    /**
     * add zero torque prior for unactuated joints
     * Keyword arguments:
            num_steps          -- number of time steps
    */
    NonlinearFactorGraph zeroTorquePriors(const std::vector<Phase> &phases) const;

    /**
     * add zero torque prior for unactuated joints
     * Keyword arguments:
            phase_seq
            phase_steps
    */
    NonlinearFactorGraph timeFactors(const std::vector<Phase> &phase_seq, const std::vector<int> phase_steps) const;


    NonlinearFactorGraph massCollocationFactors(const std::vector<Phase> &phase_seq,
                                                            const std::vector<int> &phase_steps,
                                                            const CollocationScheme collocation) const;

    NonlinearFactorGraph symmetricFactors(const std::vector<Phase> &phase_seq,
                                                          const std::vector<int> &phase_steps) const;

    /**
     * collocation factors on the base pose and twist for in-air phase
     * Keyword arguments:
            phase_seq
            phase_steps
            collocation
    */
    NonlinearFactorGraph airCollocationFactors(const std::vector<Phase> &phase_seq,
                                               const std::vector<int> &phase_steps,
                                               const CollocationScheme collocation) const;


    NonlinearFactorGraph costFactors(const size_t num_steps) const;

    /** !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! build factor graph !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

    /**
     * construct the dynamics factor graph at a phase transition step
     * Keyword arguments:
            t                  -- time step
            phase1             -- phase (transition from)
            phase2             -- phase (transition to)
    */
    NonlinearFactorGraph transitionGraph(const int t, const Phase phase1, const Phase phase2) const;

    /**
     * Construct the factor graph for the entire robot trajectory including actuators
     * Keyword arguments:
            num_steps           -- number of time steps
            dt                  -- duration for each time step
    */
    NonlinearFactorGraph trajectoryFG(const int num_steps, const double dt) const;


    NonlinearFactorGraph multiPhaseTrajectoryFG_old(
        const std::vector<Robot> &robots, const std::vector<int> &phase_steps,
        const std::vector<gtsam::NonlinearFactorGraph> &transition_graphs,
        const CollocationScheme collocation,
        const std::vector<std::vector<std::string>> &phase_joint_names) const;

    /**
     * Construct the multi-phase factor graph for the entire robot trajectory including actuators
     * the factors include:
     *      dynamics graph for each time step
     *      actuator graph (if not specify_torque)
     *      zero-torque priors for unactuated joints
     *      collocation factors between time steps
     *      guardian factors for transition step
     *      integration on time variable (t_i+1 = t_i + dt)
     *      
     *      
     * Keyword arguments:
            phases              -- the phase sequence
            phase_steps         -- number of steps for each phase
            collocation         -- collocation scheme
            specify_torque      -- if true, no pnumatic acutaotrs will be included, directly specify torque
    */
    NonlinearFactorGraph multiPhaseTrajectoryFG(const std::vector<Phase> &phases,
                                                const std::vector<int> &phase_steps,
                                                const CollocationScheme collocation,
                                                const bool specify_torque = false) const;

    /** !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! other utilities !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

    void exportPoses(double target_height, const int num_steps, const gtsam::Values& results) const;

    /** save trajectory to external files */
    void exportTrajectory(const int num_steps, const double dt, const gtsam::Values &results) const;

    /** save multi-phase trajectory to external files */
    void exportTrajectoryMultiPhase(const int num_steps, const std::vector<Phase> &phases, const gtsam::Values &results) const;

    std::vector<double> getActuatorValues(const int t, const std::vector<PneumaticActuator> &actuators, const gtsam::Values &results) const;

    void exportData(const int num_steps, const gtsam::Values &results) const;

    const DynamicsGraph &getGraphBuilder() const { return graph_builder_; };

    const std::vector<PneumaticActuator> &getActuators() const { return actuators_; };

    /** return robot with the specified pose */
    const Robot &getRobot(const Phase phase) const;

    friend class JRSimulator;
};

class JRSimulator
{
public:
    JumpingRobot &jr_;
    std::vector<JumpingRobot::Phase> phases_;
    Values results_;
    JointValueMap qs_, vs_, as_, ms_;
    double m_t_;
    gtsam::Vector6 body_twist_, body_twist_accel_;
    gtsam::Pose3 body_pose_;
    JumpingRobot::Phase phase_;
    int t_;
    double time_;
    friend class JumpingRobot;

public:
    JRSimulator(JumpingRobot &jumping_robot)
        : jr_(jumping_robot)
    {
        t_ = 0;
        time_ = 0;
        qs_ = jr_.init_angles_;
        vs_ = jr_.init_vels_;
        ms_ = jr_.init_masses_;
        phase_ = JumpingRobot::Phase::Ground;
    }

    /** refine the joint angles and vels so that they satisfy constraints */
    void refineAllJointsGround(JointValueMap& qs, JointValueMap& vs) const;

    /** compute fk, fd, check phase transition, integrate for next step, save values of this step to results_ */
    void step(double dt, const JointValueMap &torques);

    void step(double dt, const Vector &open_times, const Vector &close_times);

    /** simulate with pneumatic acutators included */
    const Values &simulate(const int num_steps, const double dt, const double init_source_pressure, const Vector &open_times, const Vector &close_times)
    {
        m_t_ = init_source_pressure * 1e3 * jr_.params_.tank_volume / (jr_.params_.Rs * jr_.params_.T);
        for (int t = 0; t <= num_steps; t++)
        {
            step(dt, open_times, close_times);
        }
        return results_;
    }

    const Values &simulateToHigh(const double dt, const double init_source_pressure, const Vector &open_times, const Vector &close_times)
    {
        m_t_ = init_source_pressure * 1e3 * jr_.params_.tank_volume / (jr_.params_.Rs * jr_.params_.T);
        int t = 0;
        while (true) {
            step(dt, open_times, close_times);
            // int j_torso = jr_.getRobot(JumpingRobot::Phase::Ground).link("l3")->id();
            auto torso_twist = results_.at<gtsam::Vector6>(internal::TwistKey(jr_.body_i_, t));
            // std::cout << "torso_twist: " << torso_twist << "\n";
            t++;
            if (phase_ == JumpingRobot::Phase::Air && torso_twist(5) < -1e-5) {
                break;
            }
        }
        return results_;
    }

    // /** simulate with constant torques */
    // const Values &simulate(const int num_steps, const double dt, const JointValueMap &torques)
    // {
    //     for (int t = 0; t <= num_steps; t++)
    //     {
    //         step(dt, torques);
    //     }
    //     return results_;
    // }

    const std::vector<JumpingRobot::Phase> &phases() const { return phases_; }

    const std::vector<int> phaseSteps() const  {
        std::vector<int> phase_steps;
        phase_steps.push_back(0);
        for (size_t i=0; i<phases_.size(); i++) {
            if (i==0) {
                // phase_steps[0] += 1;
            }
            else if (phases_[i] == phases_[i-1]) {
                phase_steps[phase_steps.size()-1] += 1;
            }
            else {
                phase_steps.push_back(1);
            }
        }
        return phase_steps;
    }
};

} // namespace gtdynamics
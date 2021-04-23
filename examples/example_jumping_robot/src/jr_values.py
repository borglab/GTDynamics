"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  jr_values.py
 * @brief Utitilities in manipulating values.
 * @author Yetong Zhang
"""

import gtdynamics as gtd
import gtsam
from gtsam import noiseModel, NonlinearFactorGraph
import numpy as np

import os, sys, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)
sys.path.insert(0, currentdir)

from jumping_robot import Actuator, JumpingRobot


class JRValues:
    """ Class of utitilities in manipulating values. """

    def __init__(self):
        """ Empty constructor. """
        return

    @staticmethod
    def copy_value(values, new_values, key):
        """ copy the value from values to new_values,
            if key does not exist, copy the value form previous step.
        """
        if values.exists(key):
            new_values.insertDouble(key, values.atDouble(key))
        else:
            new_values.insertDouble(key, values.atDouble(key-1))

    @staticmethod
    def init_config_values(jr, controls) -> gtsam.Values:
        """ Values to specify initial configuration. """
        values = gtsam.Values()

        # initial condition for source
        V_s = jr.params["pneumatic"]["v_source"]
        P_s_0 = controls["P_s_0"]
        m_s_0 = V_s * P_s_0 * 1e3 / jr.gas_constant
        m_a_0 = jr.params["pneumatic"]["init_mass"]
        values.insertDouble(Actuator.SourceVolumeKey(), V_s)
        values.insertDouble(Actuator.SourceMassKey(0), m_s_0)
        values.insertDouble(Actuator.SourcePressureKey(0), P_s_0)

        # initial joint angles nad velocities
        for joint in jr.robot.joints():
            j = joint.id()
            name = joint.name()
            q_key = gtd.internal.JointAngleKey(j, 0).key()
            v_key = gtd.internal.JointVelKey(j, 0).key()
            q = float(jr.init_config["qs"][name])
            v = float(jr.init_config["vs"][name])
            values.insertDouble(q_key, q)
            values.insertDouble(v_key, v)

        # torso pose and twists
        torso_i = jr.robot.link("torso").id()
        gtd.InsertPose(values, torso_i, 0, jr.init_config["torso_pose"])
        gtd.InsertTwist(values, torso_i, 0, jr.init_config["torso_twist"])

        # valve open close times
        for actuator in jr.actuators:
            j = actuator.j
            values.insertDouble(Actuator.MassKey(j, 0), m_a_0)
            To_key = Actuator.ValveOpenTimeKey(j)
            Tc_key = Actuator.ValveCloseTimeKey(j)
            values.insertDouble(To_key, float(controls["Tos"][name]))
            values.insertDouble(Tc_key, float(controls["Tcs"][name]))

        # time for the first step
        values.insertDouble(gtd.TimeKey(0).key(), 0)
        return values

    @staticmethod
    def get_known_values_actuator(j, k, values):
        """ Construct known values for actuator dynamics graph. """
        q_key = gtd.internal.JointAngleKey(j, k).key()
        v_key = gtd.internal.JointVelKey(j, k).key()
        m_a_key = Actuator.MassKey(j, k)

        init_values = gtsam.Values()
        init_values.insertDouble(q_key, values.atDouble(q_key))
        init_values.insertDouble(v_key, values.atDouble(v_key))
        init_values.insertDouble(m_a_key, values.atDouble(m_a_key))
        return init_values

    @staticmethod
    def init_values_from_prev_actuator(j, k, values):
        """ Construct initial values for actuator dynamics graph from
            values of previous step.
        """
        init_values = JRValues.get_known_values_actuator(j, k, values)

        P_s_key = Actuator.SourcePressureKey(k)
        P_a_key = Actuator.PressureKey(j, k)
        V_a_key = Actuator.VolumeKey(j, k)
        delta_x_key = Actuator.ContractionKey(j, k)
        f_a_key = Actuator.ForceKey(j, k)
        torque_key = gtd.internal.TorqueKey(j, k).key()

        init_values.insertDouble(P_s_key, values.atDouble(
            Actuator.SourcePressureKey(k-1)))
        init_values.insertDouble(
            P_a_key, values.atDouble(Actuator.PressureKey(j, k-1)))
        init_values.insertDouble(delta_x_key, values.atDouble(
            Actuator.ContractionKey(j, k-1)))
        init_values.insertDouble(
            f_a_key, values.atDouble(Actuator.ForceKey(j, k-1)))
        init_values.insertDouble(torque_key, values.atDouble(
            gtd.internal.TorqueKey(j, k-1).key()))
        init_values.insertDouble(
            V_a_key, values.atDouble(Actuator.VolumeKey(j, k-1)))
        return init_values

    @staticmethod
    def compute_volume(jr, delta_x):
        """ Compute actuator volume by contraction length. """
        model = noiseModel.Isotropic.Sigma(1, 0.0001)
        d_tube = jr.params["pneumatic"]["d_tube_valve_musc"] * 0.0254
        l_tube = jr.params["pneumatic"]["l_tube_valve_musc"] * 0.0254
        volume_factor = gtd.ActuatorVolumeFactor(0, 1, model, d_tube, l_tube)
        return volume_factor.computeVolume(delta_x)

    @staticmethod
    def compute_mass_flow(jr, values, j, k):
        """ Compute mass flow rate for initial estiamtes.
            Note: values should include P_s, P_a, t, To, Tc.
        """
        d_tube = jr.params["pneumatic"]["d_tube_valve_musc"] * 0.0254
        l_tube = jr.params["pneumatic"]["l_tube_valve_musc"] * 0.0254
        mu = jr.params["pneumatic"]["mu_tube"]
        ct = jr.params["pneumatic"]["time_constant_valve"]
        epsilon = jr.params["pneumatic"]["eps_tube"]
        ct = jr.params["pneumatic"]["time_constant_valve"]
        Rs = jr.params["pneumatic"]["Rs"]
        temp = jr.params["pneumatic"]["T"]
        k_const = 1.0 / (Rs * temp)

        graph = gtsam.NonlinearFactorGraph()
        P_a_key = Actuator.PressureKey(j, k)
        P_s_key = Actuator.SourcePressureKey(k)
        mdot_key = Actuator.MassRateOpenKey(j, k)
        P_s = values.atDouble(P_s_key)
        P_a = values.atDouble(P_a_key)

        prior_model = noiseModel.Isotropic.Sigma(1, 0.1)
        mass_rate_model = noiseModel.Isotropic.Sigma(1, 1e-5)

        graph.add(gtd.MassFlowRateFactor(P_a_key, P_s_key, mdot_key,
                                         mass_rate_model, d_tube, l_tube, mu,
                                         epsilon, k_const))
        graph.add(gtd.PriorFactorDouble(P_a_key, P_a, prior_model))
        graph.add(gtd.PriorFactorDouble(P_s_key, P_s, prior_model))

        init_values = gtsam.Values()
        init_values.insertDouble(P_a_key, P_a)
        init_values.insertDouble(P_s_key, P_s)
        init_values.insertDouble(mdot_key, 0.007)

        result = gtsam.LevenbergMarquardtOptimizer(
            graph, init_values).optimize()
        if graph.error(result) > 1e-5:
            params = gtsam.LevenbergMarquardtParams()
            params.setVerbosityLM("SUMMARY")
            results = gtsam.LevenbergMarquardtOptimizer(
                graph, init_values, params).optimize()
            print("error: ", graph.error(results))
            raise Exception("computing mass rate fails")

        mdot = result.atDouble(mdot_key)

        To_a_key = Actuator.ValveOpenTimeKey(j)
        Tc_a_key = Actuator.ValveCloseTimeKey(j)
        t_key = gtd.TimeKey(k).key()
        mdot_sigma_key = Actuator.MassRateActualKey(j, k)
        To = values.atDouble(To_a_key)
        Tc = values.atDouble(Tc_a_key)
        curr_time = values.atDouble(gtd.TimeKey(k).key())
        valve_control_factor = gtd.ValveControlFactor(
            t_key, To_a_key, Tc_a_key, mdot_key, mdot_sigma_key,
            mass_rate_model, ct)
        mdot_sigma = valve_control_factor.computeExpectedTrueMassFlow(
            curr_time, To, Tc, mdot)
        return mdot, mdot_sigma

    @staticmethod
    def init_values_from_init_config_actuator(jr, j, k, values):
        """ Construct initial values for actuator dynamics graph from
            actuator initial configuration.
        """
        init_values = JRValues.get_known_values_actuator(j, k, values)

        P_s_key = Actuator.SourcePressureKey(k)
        P_a_key = Actuator.PressureKey(j, k)
        V_a_key = Actuator.VolumeKey(j, k)
        delta_x_key = Actuator.ContractionKey(j, k)
        f_a_key = Actuator.ForceKey(j, k)
        torque_key = gtd.internal.TorqueKey(j, k).key()

        init_values.insertDouble(P_s_key, values.atDouble(P_s_key))
        init_values.insertDouble(P_a_key, 101.325)
        init_values.insertDouble(delta_x_key, 0.0)
        init_values.insertDouble(f_a_key, 0.0)
        init_values.insertDouble(torque_key, 0.0)
        init_values.insertDouble(V_a_key, JRValues.compute_volume(jr, 0.0))
        return init_values

    @staticmethod
    def init_values_from_prev_robot(robot, k, values):
        """ Construct initial values for robot dynamics graph from values of
            previous step.
            Note: if certain quantities are already included in `values` of
            step k, it will just include their values instead of using ones
            from previous step.
        """
        init_values = gtsam.Values()
        for joint in robot.joints():
            j = joint.id()
            q_key = gtd.internal.JointAngleKey(j, k).key()
            v_key = gtd.internal.JointVelKey(j, k).key()
            JRValues.copy_value(values, init_values, q_key)
            JRValues.copy_value(values, init_values, v_key)
            gtd.InsertJointAccelDouble(
                init_values, j, k, gtd.JointAccelDouble(values, j, k-1))
            i1 = joint.parent().id()
            i2 = joint.child().id()
            gtd.InsertWrench(init_values, i1, j, k,
                             gtd.Wrench(values, i1, j, k-1))
            gtd.InsertWrench(init_values, i2, j, k,
                             gtd.Wrench(values, i2, j, k-1))
            torque_key = gtd.internal.TorqueKey(j, k).key()
            JRValues.copy_value(values, init_values, torque_key)

        for link in robot.links():
            i = link.id()
            if values.exists(gtd.internal.PoseKey(i, k).key()):
                gtd.InsertPose(init_values, i, k, gtd.Pose(values, i, k))
            else:
                gtd.InsertPose(init_values, i, k, gtd.Pose(values, i, k-1))
            if values.exists(gtd.internal.TwistKey(i, k).key()):
                gtd.InsertTwist(init_values, i, k, gtd.Twist(values, i, k))
            else:
                gtd.InsertTwist(init_values, i, k, gtd.Twist(values, i, k-1))
            gtd.InsertTwistAccel(
                init_values, i, k, gtd.TwistAccel(values, i, k-1))
        return init_values

    @staticmethod
    def init_values_from_fk_robot(jr, k, values):
        """ Construct initial values for dynamics graph, with forward kinematics,
            with unknowns as zeros.
            Note: if certain quantities already included in `values` of step k,
            it will just include their values instead of computing new ones.
        """

        # perform forward kinematics
        link_names = [link.name() for link in jr.robot.links()]
        # gtd.DynamicsGraph.printValues(values)
        if "ground" not in link_names:
            fk_results = jr.robot.forwardKinematics(values, k, "torso")
        else:
            fk_results = jr.robot.forwardKinematics(values, k)

        init_values = gtsam.Values()
        for link in jr.robot.links():
            i = link.id()
            pose_key = gtd.internal.PoseKey(i, k).key()
            twist_key = gtd.internal.TwistKey(i, k).key()
            pose = fk_results.atPose3(pose_key)
            if values.exists(pose_key):
                pose = gtd.Pose(values, i, k)
            twist = gtd.Twist(fk_results, i, k)
            if values.exists(twist_key):
                twist = gtd.Twist(values, i, k)
            gtd.InsertPose(init_values, i, k, pose)
            gtd.InsertTwist(init_values, i, k, twist)
            gtd.InsertTwistAccel(init_values, i, k, np.zeros(6))

        # assign zeros to unknown values
        for joint in jr.robot.joints():
            j = joint.id()
            q_key = gtd.internal.JointAngleKey(j, k).key()
            v_key = gtd.internal.JointVelKey(j, k).key()
            q = 0.0
            v = 0.0
            if values.exists(q_key):
                q = values.atDouble(q_key)
            if values.exists(v_key):
                v = values.atDouble(v_key)
            init_values.insertDouble(q_key, q)
            init_values.insertDouble(v_key, v)
            gtd.InsertJointAccelDouble(init_values, j, k, 0.0)
            i1 = joint.parent().id()
            i2 = joint.child().id()
            gtd.InsertWrench(init_values, i1, j, k, np.zeros(6))
            gtd.InsertWrench(init_values, i2, j, k, np.zeros(6))
            torque_key = gtd.internal.TorqueKey(j, k).key()
            if values.exists(torque_key):
                gtd.InsertTorqueDouble(
                    init_values, j, k, values.atDouble(torque_key))
            else:
                gtd.InsertTorqueDouble(init_values, j, k, 0.0)

        return init_values

    @staticmethod
    def integrate_joints(jr, values, k, dt):
        """ Integrate joint angle and velocity and add to values. """
        for joint in jr.robot.joints():
            j = joint.id()
            q_prev = gtd.JointAngleDouble(values, j, k-1)
            v_prev = gtd.JointVelDouble(values, j, k-1)
            a_prev = gtd.JointAccelDouble(values, j, k-1)
            v_curr = v_prev + a_prev * dt
            q_curr = q_prev + v_prev * dt + 0.5 * a_prev * dt * dt
            gtd.InsertJointAngleDouble(values, j, k, q_curr)
            gtd.InsertJointVelDouble(values, j, k, v_curr)

    @staticmethod
    def integrate_torso(jr, values, k, dt):
        """ Integrate pose and twist of torso link and add to values. """
        torso_i = jr.robot.link("torso").id()
        pose_torso_prev = gtd.Pose(values, torso_i, k-1)
        twist_torso_prev = gtd.Twist(values, torso_i, k-1)
        twistaccel_torso_prev = gtd.TwistAccel(values, torso_i, k-1)
        twist_torso_curr = twist_torso_prev + twistaccel_torso_prev * dt
        prevTcurr = gtsam.Pose3.Expmap(
            dt * twist_torso_prev + 0.5*twistaccel_torso_prev * dt * dt)
        pose_torso_curr = pose_torso_prev.compose(prevTcurr)
        gtd.InsertPose(values, torso_i, k, pose_torso_curr)
        gtd.InsertTwist(values, torso_i, k, twist_torso_curr)

    @staticmethod
    def integrate_mass(jr, values, k, dt):
        """ Integrate air mass and add to values. """
        total_m_out = 0
        for actuator in jr.actuators:
            j = actuator.j
            m_a_prev = values.atDouble(Actuator.MassKey(j, k-1))
            mdot_a_prev = values.atDouble(Actuator.MassRateActualKey(j, k-1))
            m_a_curr = m_a_prev + mdot_a_prev * dt
            values.insertDouble(Actuator.MassKey(j, k), m_a_curr)
            total_m_out += mdot_a_prev * dt
        m_s_prev = values.atDouble(Actuator.SourceMassKey(k-1))
        m_s_curr = m_s_prev - total_m_out
        values.insertDouble(Actuator.SourceMassKey(k), m_s_curr)

    @staticmethod
    def get_ground_force_z(jr, side, k, values):
        """ Get ground reaction force for the specifed foot contact. """
        i = jr.robot.link("shank_" + side).id()
        j = jr.robot.joint("foot_" + side).id()
        wrench_b = gtd.Wrench(values, i, j, k)
        T_wb = gtd.Pose(values, i, k)
        wrench_w = T_wb.inverse().AdjointMap().transpose().dot(wrench_b)
        print(side + " force: ", wrench_w[5])
        return wrench_w[5]

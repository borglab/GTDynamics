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



import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir) 
sys.path.insert(0,currentdir) 

import gtdynamics as gtd
import gtsam
from gtsam import noiseModel, NonlinearFactorGraph
import numpy as np

from jumping_robot import Actuator, JumpingRobot


class JRValues:
    def __init__(self):
        return
    
    @staticmethod
    def get_known_values_actuator(j, k, values):
        """ Construct known values for actuator dynamics graph.
        """
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
        """ Construct initial values for actuator dynamics graph from values of previous step.
        """
        init_values = JRValues.get_known_values_actuator(j, k, values)

        P_s_key = Actuator.SourcePressureKey(k)
        P_a_key = Actuator.PressureKey(j, k)
        V_a_key = Actuator.VolumeKey(j, k)
        delta_x_key = Actuator.ContractionKey(j, k)
        f_a_key = Actuator.ForceKey(j, k)
        torque_key = gtd.internal.TorqueKey(j, k).key()
        
        init_values.insertDouble(P_s_key, values.atDouble(Actuator.SourcePressureKey(k-1)))
        init_values.insertDouble(P_a_key, values.atDouble(Actuator.PressureKey(j, k-1)))
        init_values.insertDouble(delta_x_key, values.atDouble(Actuator.ContractionKey(j, k-1)))
        init_values.insertDouble(f_a_key, values.atDouble(Actuator.ForceKey(j, k-1)))
        init_values.insertDouble(torque_key, values.atDouble(gtd.internal.TorqueKey(j, k-1).key()))
        init_values.insertDouble(V_a_key, values.atDouble(Actuator.VolumeKey(j, k-1)))
        return init_values

    @staticmethod
    def compute_volume(jr, delta_x):
        model = noiseModel.Isotropic.Sigma(1, 0.0001)
        d_tube = jr.params["pneumatic"]["d_tube_valve_musc"] * 0.0254
        l_tube = jr.params["pneumatic"]["l_tube_valve_musc"] * 0.0254
        volume_factor = gtd.ActuatorVolumeFactor(0, 1, model, d_tube, l_tube)
        return volume_factor.computeVolume(delta_x)

    @staticmethod
    def compute_mass_flow(jr, values, j, k):
        model = noiseModel.Isotropic.Sigma(1, 0.0001)
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

        graph.add(gtd.MassFlowRateFactor(P_a_key, P_s_key, mdot_key, mass_rate_model, d_tube, l_tube, mu, epsilon, k_const))
        graph.add(gtd.PriorFactorDouble(P_a_key, P_a, prior_model))
        graph.add(gtd.PriorFactorDouble(P_s_key, P_s, prior_model))

        init_values = gtsam.Values()
        init_values.insertDouble(P_a_key, P_a)
        init_values.insertDouble(P_s_key, P_s)
        init_values.insertDouble(mdot_key, 0.00665209781339859)

        result = gtsam.LevenbergMarquardtOptimizer(graph, init_values).optimize()
        if graph.error(result)>1e-5:
            params = gtsam.LevenbergMarquardtParams()
            params.setVerbosityLM("SUMMARY")
            results = gtsam.LevenbergMarquardtOptimizer(graph, init_values, params).optimize()
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
        valve_control_factor = gtd.ValveControlFactor(t_key, To_a_key, Tc_a_key, mdot_key, mdot_sigma_key, mass_rate_model, ct)
        mdot_sigma = valve_control_factor.computeExpectedTrueMassFlow(curr_time, To, Tc, mdot)
        return mdot, mdot_sigma


    @staticmethod
    def init_values_from_init_config(jr, j, k, values):
        """ Construct initial values for actuator dynamics graph from actuator initial configuration.
        """
        init_values = JRValues.get_known_values_actuator(j, k, values)

        P_s_key = Actuator.SourcePressureKey(k)
        P_a_key = Actuator.PressureKey(j, k)
        V_a_key = Actuator.VolumeKey(j, k)
        delta_x_key = Actuator.ContractionKey(j, k)
        f_a_key = Actuator.ForceKey(j, k)
        torque_key = gtd.internal.TorqueKey(j, k).key()
        mdot_key = Actuator.MassRateOpenKey(j, k)
        mdot_sigma_key = Actuator.MassRateActualKey(j, k)

        init_values.insertDouble(P_s_key, values.atDouble(P_s_key))
        init_values.insertDouble(P_a_key, 101.325)
        init_values.insertDouble(delta_x_key, 0.0)
        init_values.insertDouble(f_a_key, 0.0)
        init_values.insertDouble(torque_key, 0.0)
        init_values.insertDouble(V_a_key, JRValues.compute_volume(jr, 0.0))
        return init_values
        

    @staticmethod
    def init_values_from_prev_robot(robot, k, values):
        """ Construct initial values for robot dynamics graph from values of previous step.
        """
        init_values = gtsam.Values()
        for joint in robot.joints():
            j = joint.id()
            if values.exists(gtd.internal.JointAngleKey(j, k).key()):
                gtd.InsertJointAngleDouble(init_values, j, k, gtd.JointAngleDouble(values, j, k))
            else:
                gtd.InsertJointAngleDouble(init_values, j, k, gtd.JointAngleDouble(values, j, k-1))

            if values.exists(gtd.internal.JointVelKey(j, k).key()):
                gtd.InsertJointVelDouble(init_values, j, k, gtd.JointVelDouble(values, j, k))
            else:
                gtd.InsertJointVelDouble(init_values, j, k, gtd.JointVelDouble(values, j, k-1))
            gtd.InsertJointAccelDouble(init_values, j, k, gtd.JointAccelDouble(values, j, k-1))
            i1 = joint.parent().id()
            i2 = joint.child().id()
            gtd.InsertWrench(init_values, i1, j, k, gtd.Wrench(values, i1, j, k-1))
            gtd.InsertWrench(init_values, i2, j, k, gtd.Wrench(values, i2, j, k-1))
            if values.exists(gtd.internal.TorqueKey(j, k).key()):
                gtd.InsertTorqueDouble(init_values, j, k, gtd.TorqueDouble(values, j, k))
            else:
                gtd.InsertTorqueDouble(init_values, j, k, gtd.TorqueDouble(values, j, k-1))
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
            gtd.InsertTwistAccel(init_values, i, k, gtd.TwistAccel(values, i, k-1))
        return init_values

    @staticmethod
    def init_values_from_fk_robot(jr, k, values):
        """ Construct initial values with unknowns as zeros.
        """

        # # perform forward kinematics
        link_names = [link.name() for link in jr.robot.links()]
        if not "ground" in link_names:
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
                gtd.InsertTorqueDouble(init_values, j, k, values.atDouble(torque_key))
            else:
                gtd.InsertTorqueDouble(init_values, j, k, 0.0)

        return init_values


if __name__ == "__main__":
    yaml_file_path = "examples/example_jumping_robot/yaml/robot_config.yaml"
    jr = JumpingRobot(yaml_file_path, JumpingRobot.create_init_config())

    j = 1
    k = 0

    values = gtsam.Values()
    values.insertDouble(Actuator.ValveOpenTimeKey(j), 0)
    values.insertDouble(Actuator.ValveCloseTimeKey(j), 1)
    P_a_key = Actuator.PressureKey(j, k)
    P_s_key = Actuator.SourcePressureKey(k)
    t_key = gtd.TimeKey(k).key()
    values.insertDouble(P_a_key, 101.325)
    values.insertDouble(P_s_key, 65 * 6894.76/1000)
    values.insertDouble(t_key, 0.5)

    mdot, mdot_sigma = JRValues.compute_mass_flow(jr, values, j, k)
    print(mdot, mdot_sigma)
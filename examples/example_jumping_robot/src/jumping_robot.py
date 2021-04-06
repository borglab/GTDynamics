"""
@file   jumping_robot.py
@brief  create jumping robot from yaml specificaiton file
@author Yetong Zhang
"""


import yaml
import numpy as np
import gtsam
import gtdynamics as gtd


class Actuator:
    """ class that stores all parameters for an actuator """

    def __init__(self, name, robot, actuator_config, positive):
        self.name = name
        self.j = robot.joint(name).id()
        self.config = actuator_config
        self.positive = positive

    """ actuator specific keys
        all keys use the same naming convention as paper
    """
    @staticmethod
    def PressureKey(j, t):
        return gtd.DynamicsSymbol.JointSymbol("Pa", j, t).key()

    @staticmethod
    def SourcePressureKey(t):
        return gtd.DynamicsSymbol.SimpleSymbol("Ps", t).key()

    @staticmethod
    def ContractionKey(j, t):
        return gtd.DynamicsSymbol.JointSymbol("dx", j, t).key()

    @staticmethod
    def ForceKey(j, t):
        return gtd.DynamicsSymbol.JointSymbol("fa", j, t).key()

    @staticmethod
    def MassKey(j, t):
        return gtd.DynamicsSymbol.JointSymbol("ma", j, t).key()

    @staticmethod
    def SourceMassKey(t):
        return gtd.DynamicsSymbol.SimpleSymbol("ms", t).key()

    @staticmethod
    def MassRateOpenKey(j, t):
        return gtd.DynamicsSymbol.JointSymbol("mo", j, t).key()

    @staticmethod
    def MassRateActualKey(j, t):
        return gtd.DynamicsSymbol.JointSymbol("md", j, t).key()

    @staticmethod
    def VolumeKey(j, t):
        return gtd.DynamicsSymbol.JointSymbol("Va", j, t).key()

    @staticmethod
    def SourceVolumeKey():
        return gtd.DynamicsSymbol.SimpleSymbol("Vs", 0).key()

    @staticmethod
    def ValveOpenTimeKey(j):
        return gtd.DynamicsSymbol.JointSymbol("To", j, 0).key()

    @staticmethod
    def ValveCloseTimeKey(j):
        return gtd.DynamicsSymbol.JointSymbol("Tc", j, 0).key()


class JumpingRobot:
    """ class that stores a GTDynamics robot class and all parameters for 
        a jumping robot """

    def __init__(self, yaml_file_path, init_config):
        self.params = self.load_file(yaml_file_path)
        self.init_config = init_config
        self.robot = self.create_robot(self.params)
        self.actuators = [Actuator("knee_r", self.robot, self.params["knee"], False),
                          Actuator("hip_r", self.robot,
                                   self.params["hip"], True),
                          Actuator("hip_l", self.robot,
                                   self.params["hip"], True),
                          Actuator("knee_l", self.robot, self.params["knee"], False)]

        Rs = self.params["pneumatic"]["Rs"]
        temperature = self.params["pneumatic"]["T"]
        self.gas_constant = Rs * temperature

    @staticmethod
    def load_file(yaml_file_path: str):
        """ load jumping robot params from yaml file """
        with open(yaml_file_path) as file:
            params = yaml.load(file, Loader=yaml.FullLoader)
            return params

    @staticmethod
    def create_init_config(rest_angles=[0, 0, 0, 0, 0, 0],
                           init_angles=[0, 0, 0, 0, 0, 0],
                           init_vels=[0, 0, 0, 0, 0, 0]):
        """ create initial configuraiton specification

        Args:
            rest_angles (list, optional): joint angles at rest.
            init_angles (list, optional): initial joint angles.
            init_vels (list, optional): initial joint velocities.

        Returns:
            Dict: specifiction of initial configuration
        """
        joint_names = ["foot_r", "knee_r",
                       "hip_r", "hip_l", "knee_l", "foot_l"]
        init_config = {}
        init_config["qs"] = {}
        init_config["vs"] = {}
        init_config["qs_rest"] = {}
        for joint_name, joint_angle in zip(joint_names, init_angles):
            init_config["qs"][joint_name] = joint_angle
        for joint_name, joint_vel in zip(joint_names, init_vels):
            init_config["vs"][joint_name] = joint_vel
        for joint_name, rest_angle in zip(joint_names, rest_angles):
            init_config["qs_rest"][joint_name] = rest_angle
        return init_config

    @staticmethod
    def create_controls(Tos=[0, 0, 0, 0], Tcs=[0, 0, 0, 0], P_s_0=0.0):
        """ create control specficiation for jumping robot

        Args:
            Tos (list, optional): valve open times. 
            Tcs (list, optional): valve close times. 
            P_s_0 (float, optional): initial tank pressure.

        Returns:
            Dict: specification of controls
        """
        actuator_names = ["knee_r", "hip_r", "hip_l", "knee_l"]
        controls = {}
        controls["Tos"] = {}
        controls["Tcs"] = {}
        controls["P_s_0"] = P_s_0
        for actuator_name, To in zip(actuator_names, Tos):
            controls["Tos"][actuator_name] = To
        for actuator_name, Tc in zip(actuator_names, Tcs):
            controls["Tcs"][actuator_name] = Tc
        return controls

    @staticmethod
    def create_robot(params) -> gtd.Robot:
        """ create the robot

        Args:
            params (Dict): jumping robot parameters

        Returns:
            gtd.Robot
        """
        # morphology parameters
        length_list = params["morphology"]["l"]
        mass_list = params["morphology"]["m"]
        link_radius = params["morphology"]["r_cyl"]
        foot_distance = params["morphology"]["foot_dist"]

        # compute link, joint poses
        link_poses, joint_poses = JumpingRobot.compute_poses(length_list, foot_distance)
        
        # cosntruct links
        ground = gtd.Link(0, "ground", 1, np.eye(3), gtsam.Pose3(), gtsam.Pose3(), True)
        shank_r = JumpingRobot.construct_link(
            1, "shank_r", mass_list[4], length_list[4], link_radius, link_poses["shank_r"])
        thigh_r = JumpingRobot.construct_link(
            2, "thigh_r", mass_list[3], length_list[3], link_radius, link_poses["thigh_r"])
        torso = JumpingRobot.construct_link(
            3, "torso", mass_list[2], length_list[2], link_radius, link_poses["torso"])
        thigh_l = JumpingRobot.construct_link(
            4, "thigh_l", mass_list[1], length_list[1], link_radius, link_poses["thigh_l"])
        shank_l = JumpingRobot.construct_link(
            5, "shank_l", mass_list[0], length_list[0], link_radius, link_poses["shank_l"])

        # construct joints
        axis_r = np.array([1, 0, 0])
        axis_l = np.array([-1, 0, 0])
        foot_r = gtd.RevoluteJoint(
            0, "foot_r", joint_poses["foot_r"], ground, shank_r, gtd.JointParams(), axis_r)
        knee_r = gtd.RevoluteJoint(
            1, "knee_r", joint_poses["knee_r"], shank_r, thigh_r, gtd.JointParams(), axis_r)
        hip_r = gtd.RevoluteJoint(
            2, "hip_r", joint_poses["hip_r"], thigh_r, torso, gtd.JointParams(), axis_r)
        hip_l = gtd.RevoluteJoint(
            3, "hip_l", joint_poses["hip_l"], thigh_l, torso, gtd.JointParams(), axis_l)
        knee_l = gtd.RevoluteJoint(
            4, "knee_l", joint_poses["knee_l"], shank_l, thigh_l, gtd.JointParams(), axis_l)
        foot_l = gtd.RevoluteJoint(
            5, "foot_l", joint_poses["foot_l"], ground, shank_l, gtd.JointParams(), axis_l)

        # use links, joints to create robot
        links = [ground, shank_r, thigh_r, torso, thigh_l, shank_l]
        joints = [foot_r, knee_r, hip_r, hip_l, knee_l, foot_l]

        # TODO(yetong): make Robot constructor simpler by directly taking lists
        # (attach joints to links)
        for joint in joints:
            joint.parent().addJoint(joint)
            joint.child().addJoint(joint)

        link_dict = {}
        joint_dict = {}
        for joint in joints:
            joint_dict[joint.name()] = joint
        for link in links:
            link_dict[link.name()] = link
        return gtd.Robot(link_dict, joint_dict)

    @staticmethod
    def compute_poses(length_list: list, foot_distance: float):
        """ Compute poses for links and joints. (Assume the shank link
        and the thigh link of a leg are on the same line.)

        Args:
            length_list (list): length of links
            foot_distance (float): distance between two feet

        Returns:
            dict, dict: link poses, joint poses
        """

        # compute the configuration by solving a simple factor graph:
        #   - under the constraint that thigh and shank are in the same line,
        #   the robot can be simplied as 3 links
        #   - 0 represents the point of right foot contact
        #   - 1 represents the point of right hip joint
        #   - 2 represents the point of left hip joint
        #   - 3 represents the point of left foot contact
        length_right = length_list[-1] + length_list[-2]
        length_left = length_list[0] + length_list[1]
        length_mid = length_list[2]

        init_values = gtsam.Values()
        init_values.insert(0, gtsam.Pose2(foot_distance/2, 0, 0))
        init_values.insert(1, gtsam.Pose2(foot_distance/2, length_right, 0))
        init_values.insert(2, gtsam.Pose2(-foot_distance/2, length_left, 0))
        init_values.insert(3, gtsam.Pose2(-foot_distance/2, 0, 0))

        range_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1]))
        prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1, 1, 1]))
        graph = gtsam.NonlinearFactorGraph()
        graph.add(gtsam.RangeFactorPose2(0, 1, length_right, range_noise))
        graph.add(gtsam.RangeFactorPose2(1, 2, length_mid, range_noise))
        graph.add(gtsam.RangeFactorPose2(2, 3, length_left, range_noise))
        graph.add(gtsam.PriorFactorPose2(
            0, gtsam.Pose2(foot_distance/2, 0, 0), prior_noise))
        graph.add(gtsam.PriorFactorPose2(
            3, gtsam.Pose2(-foot_distance/2, 0, 0), prior_noise))

        result = gtsam.LevenbergMarquardtOptimizer(
            graph, init_values).optimize()
        p0 = result.atPose2(0)
        p1 = result.atPose2(1)
        p2 = result.atPose2(2)
        p3 = result.atPose2(3)
        rot_r = gtsam.Rot3.Rx(np.arctan2(p1.y() - p0.y(), p1.x() - p0.x()))
        rot_m = gtsam.Rot3.Rx(np.arctan2(p2.y() - p1.y(), p2.x() - p1.x()))
        rot_l = gtsam.Rot3.Rx(np.arctan2(p3.y() - p2.y(), p3.x() - p2.x()))

        # use the optimization result of 4 points to initialize the poses
        # of links and joints
        link_poses = {}
        link_poses["shank_r"] = gtsam.Pose3(
            rot_r, gtsam.Point3(0, p0.x(), p0.y()))
        link_poses["thigh_r"] = gtsam.Pose3(rot_r, gtsam.Point3(
            0, (p0.x() + p1.x())/2, (p0.y() + p1.y())/2))
        link_poses["torso"] = gtsam.Pose3(
            rot_m, gtsam.Point3(0, p1.x(), p1.y()))
        link_poses["thigh_l"] = gtsam.Pose3(
            rot_l, gtsam.Point3(0, p2.x(), p2.y()))
        link_poses["shank_l"] = gtsam.Pose3(rot_l, gtsam.Point3(
            0, (p2.x() + p3.x())/2, (p2.y()+p3.y())/2))

        joint_poses = {}
        joint_poses["foot_r"] = gtsam.Pose3(
            gtsam.Rot3(), gtsam.Point3(0, p0.x(), p0.y()))
        joint_poses["knee_r"] = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(
            0, (p0.x() + p1.x())/2, (p0.y() + p1.y())/2))
        joint_poses["hip_r"] = gtsam.Pose3(
            gtsam.Rot3(), gtsam.Point3(0, p1.x(), p1.y()))
        joint_poses["hip_l"] = gtsam.Pose3(
            gtsam.Rot3(), gtsam.Point3(0, p2.x(), p2.y()))
        joint_poses["knee_l"] = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(
            0, (p2.x() + p3.x())/2, (p2.y()+p3.y())/2))
        joint_poses["foot_l"] = gtsam.Pose3(
            gtsam.Rot3(), gtsam.Point3(0, p3.x(), p3.y()))
        return link_poses, joint_poses

    @staticmethod
    def construct_link(link_id: int, link_name: str, mass: float, length: float, radius: float, pose: gtsam.Pose3):
        """ construct a link """
        lTcom = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, length/2, 0))
        inertia = JumpingRobot.get_link_inertia(mass, length, radius)
        return gtd.Link(link_id, link_name, mass, inertia, pose, lTcom, False)

    @staticmethod
    def get_link_inertia(mass: float, length: float, radius: float):
        """ compute inertia matrix for a link """
        Ixx = 1/12 * mass * (3*radius**2 + length**2)
        Iyy = 1/2 * mass * radius**2
        Izz = Ixx
        inertia = np.diag([Ixx, Iyy, Izz])
        return inertia

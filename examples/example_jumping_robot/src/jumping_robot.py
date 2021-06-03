"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  jumping_robot.py
 * @brief Create jumping robot from yaml specificaiton file.
 * @author Yetong Zhang
"""


import yaml
import numpy as np
import gtsam
from gtsam import Pose2, Pose3, Point3, Rot3, Values
from gtsam import NonlinearFactorGraph, RangeFactorPose2, PriorFactorPose2
import gtdynamics as gtd


class Actuator:
    """ Class that stores all parameters for an actuator. """

    def __init__(self, name, robot, actuator_config, positive):
        self.name = name
        self.j = robot.joint(name).id()
        self.config = actuator_config
        self.positive = positive

    """ actuator specific keys: 
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

    @staticmethod
    def DampingKey():
        return gtd.DynamicsSymbol.SimpleSymbol('bd', 0).key()
    
    @staticmethod
    def TendonStiffnessKey(j):
        if j==1 or j==4:
            return gtd.DynamicsSymbol.JointSymbol("kt", 1, 0).key()
        elif j==2 or j==3:
            return gtd.DynamicsSymbol.JointSymbol("kt", 2, 0).key()
        else:
            raise Exception("not a joint with actuator")
    
    @staticmethod
    def TubeDiameterKey():
        return gtd.DynamicsSymbol.SimpleSymbol("Dt", 0).key()


class JumpingRobot:
    """ Class that stores a GTDynamics robot class and all parameters for 
        a jumping robot. """

    def __init__(self, params, init_config, phase=0):
        """ Constructor

        Args:
            params (dict): parameters for jumping robot
            init_config (dict): initial configuration
            phase (int, optional): phase Defaults to 0
                - 0: ground
                - 1: left on ground
                - 2: right on ground
                - 3: in air
        """
        self.params = params
        self.init_config = init_config
        self.robot = self.create_robot(self.params, init_config, phase)
        self.actuators = [Actuator("knee_r", self.robot, self.params["knee"], False),
                          Actuator("hip_r", self.robot, self.params["hip"], True),
                          Actuator("hip_l", self.robot, self.params["hip"], True),
                          Actuator("knee_l", self.robot, self.params["knee"], False)]
        self.marker_locations = JumpingRobot.get_marker_locations()
        self.calibration = JumpingRobot.get_camera_calibration(self.params['cam_params'])

        Rs = self.params["pneumatic"]["Rs"]
        temperature = self.params["pneumatic"]["T"]
        self.gas_constant = Rs * temperature

    @staticmethod
    def from_yaml(yaml_file_path, init_config, phase=0):
        """ Create jumping robot from yaml file. """
        params = JumpingRobot.load_file(yaml_file_path)
        return JumpingRobot(params, init_config, phase)

    def jr_with_phase(self, phase):
        """ Create the robot with same params but of different phase. """
        return JumpingRobot(self.params, self.init_config, phase)

    @staticmethod
    def load_file(yaml_file_path: str):
        """ Load jumping robot params from yaml file. """
        with open(yaml_file_path) as file:
            params = yaml.load(file, Loader=yaml.FullLoader)
            return params

    @staticmethod
    def create_init_config(torso_pose=gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, 0, 1.1)),
                           torso_twist=np.zeros(6),
                           rest_angles=[0, 0, 0, 0, 0, 0],
                           init_angles=[0, 0, 0, 0, 0, 0],
                           init_vels=[0, 0, 0, 0, 0, 0],
                           P_s_0=0.0,
                           foot_dist=0.55):
        """ Create initial configuration specification.

        Args:
            torso_pose (gtsam.Pose3, optional): torso pose
            torso_twist (Vector6, optional): torso twist
            rest_angles (list, optional): joint angles at rest.
            init_angles (list, optional): initial joint angles.
            init_vels (list, optional): initial joint velocities.
            P_s_0 (float, optional): initial tank pressure.
            foot_dist (double): distance between two feet

        Returns:
            Dict: specifiction of initial configuration
        """
        joint_names = ["foot_r", "knee_r",
                       "hip_r", "hip_l", "knee_l", "foot_l"]
        init_config = {}
        init_config["torso_pose"] = torso_pose
        init_config["torso_twist"] = torso_twist
        init_config["P_s_0"] = P_s_0
        init_config["qs"] = {}
        init_config["vs"] = {}
        init_config["qs_rest"] = {}
        for joint_name, joint_angle in zip(joint_names, init_angles):
            init_config["qs"][joint_name] = joint_angle
        for joint_name, joint_vel in zip(joint_names, init_vels):
            init_config["vs"][joint_name] = joint_vel
        for joint_name, rest_angle in zip(joint_names, rest_angles):
            init_config["qs_rest"][joint_name] = rest_angle
        init_config["foot_dist"] = foot_dist
        return init_config

    @staticmethod
    def create_controls(Tos=[0, 0, 0, 0], Tcs=[0, 0, 0, 0]):
        """ Create control specficiation for jumping robot.

        Args:
            Tos (list, optional): valve open times. 
            Tcs (list, optional): valve close times. 

        Returns:
            Dict: specification of controls
        """
        actuator_names = ["knee_r", "hip_r", "hip_l", "knee_l"]
        controls = {}
        controls["Tos"] = {}
        controls["Tcs"] = {}
        for actuator_name, To in zip(actuator_names, Tos):
            controls["Tos"][actuator_name] = To
        for actuator_name, Tc in zip(actuator_names, Tcs):
            controls["Tcs"][actuator_name] = Tc
        return controls

    @staticmethod
    def create_robot(params, init_config, phase) -> gtd.Robot:
        """ Create the robot. """
        # morphology parameters
        length_list = params["morphology"]["l"]
        mass_list = params["morphology"]["m"]
        link_radius = params["morphology"]["r_cyl"]
        foot_dist = init_config["foot_dist"]

        # compute link, joint poses
        link_poses, joint_poses = JumpingRobot.compute_poses(length_list, foot_dist)

        # cosntruct links
        ground = gtd.Link(0, "ground", 1, np.eye(3), Pose3(), Pose3(), True)
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
        if phase == 0:
            links = [ground, shank_r, thigh_r, torso, thigh_l, shank_l]
            joints = [foot_r, knee_r, hip_r, hip_l, knee_l, foot_l]
        elif phase == 1:
            links = [ground, shank_r, thigh_r, torso, thigh_l, shank_l]
            joints = [knee_r, hip_r, hip_l, knee_l, foot_l]
        elif phase == 2:
            links = [ground, shank_r, thigh_r, torso, thigh_l, shank_l]
            joints = [foot_r, knee_r, hip_r, hip_l, knee_l]
        elif phase == 3:
            links = [shank_r, thigh_r, torso, thigh_l, shank_l]
            joints = [knee_r, hip_r, hip_l, knee_l]
        else:
            raise Exception("no such phase " + str(phase))

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
    def compute_poses(length_list: list, foot_dist: float):
        """ Compute poses for links and joints. (Assume the shank link
        and the thigh link of a leg are on the same line.)

        Args:
            length_list (list): length of links
            foot_dist (float): distance between two feet

        Returns:
            dict, dict: link poses, joint poses
        """

        # compute the configuration of the robot by solving a small optimization problem
        values = JumpingRobot.compute_poses_helper(length_list, foot_dist)
        p0 = values.atPose2(0)
        p1 = values.atPose2(1)
        p2 = values.atPose2(2)
        p3 = values.atPose2(3)
        rot_r = Rot3.Rx(np.arctan2(p1.y() - p0.y(), p1.x() - p0.x()) - np.pi/2)
        rot_m = Rot3.Rx(np.arctan2(p1.y() - p2.y(), p1.x() - p2.x()))
        rot_l = Rot3.Rx(np.arctan2(p2.y() - p3.y(), p2.x() - p3.x()) - np.pi/2)

        # use the optimization result of 4 points to initialize the poses
        # of links and joints
        link_poses = {}
        link_poses["shank_r"] = Pose3(rot_r, Point3(0, p0.x(), p0.y()))
        link_poses["thigh_r"] = Pose3(rot_r, Point3(0, (p0.x() + p1.x())/2, (p0.y() + p1.y())/2))
        link_poses["torso"] = Pose3(rot_m, Point3(0, p2.x(), p2.y()))
        link_poses["thigh_l"] = Pose3(rot_l, Point3(0, (p2.x() + p3.x())/2, (p2.y()+p3.y())/2))
        link_poses["shank_l"] = Pose3(rot_l, Point3(0, p3.x(), p3.y()))

        joint_poses = {}
        joint_poses["foot_r"] = Pose3(Rot3(), Point3(0, p0.x(), p0.y()))
        joint_poses["knee_r"] = Pose3(Rot3(), Point3(0, (p0.x() + p1.x())/2, (p0.y() + p1.y())/2))
        joint_poses["hip_r"] = Pose3(Rot3(), Point3(0, p1.x(), p1.y()))
        joint_poses["hip_l"] = Pose3(Rot3(), Point3(0, p2.x(), p2.y()))
        joint_poses["knee_l"] = Pose3(Rot3(), Point3(0, (p2.x() + p3.x())/2, (p2.y()+p3.y())/2))
        joint_poses["foot_l"] = Pose3(Rot3(), Point3(0, p3.x(), p3.y()))
        return link_poses, joint_poses

    @staticmethod
    def compute_poses_helper(length_list: list, foot_dist: float) -> Values:
        """ compute the configuration by solving a simple factor graph:
            - under the constraint that thigh and shank are in the same line,
                the robot can be simplied as 3 links
            - 0 represents the point of right foot contact
            - 1 represents the point of right hip joint
            - 2 represents the point of left hip joint
            - 3 represents the point of left foot contact
        """
        length_right = length_list[-1] + length_list[-2]
        length_left = length_list[0] + length_list[1]
        length_mid = length_list[2]

        init_values = Values()
        init_values.insert(0, Pose2(foot_dist/2, 0, 0))
        init_values.insert(1, Pose2(foot_dist/2, length_right, 0))
        init_values.insert(2, Pose2(-foot_dist/2, length_left, 0))
        init_values.insert(3, Pose2(-foot_dist/2, 0, 0))

        range_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1]))
        prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1, 1, 1]))
        graph = NonlinearFactorGraph()
        graph.add(RangeFactorPose2(0, 1, length_right, range_noise))
        graph.add(RangeFactorPose2(1, 2, length_mid, range_noise))
        graph.add(RangeFactorPose2(2, 3, length_left, range_noise))
        graph.add(PriorFactorPose2(0, Pose2(foot_dist/2, 0, 0), prior_noise))
        graph.add(PriorFactorPose2(3, Pose2(-foot_dist/2, 0, 0), prior_noise))

        return gtsam.LevenbergMarquardtOptimizer(graph, init_values).optimize()

    @staticmethod
    def construct_link(link_id: int, link_name: str, mass: float, length: float, radius: float, pose: Pose3):
        """ Construct a link. """
        if link_name == "torso":
            lTcom = Pose3(Rot3(), Point3(0, length/2, 0))
        else:
            lTcom = Pose3(Rot3(), Point3(0, 0, length/2))
        inertia = JumpingRobot.compute_link_inertia(mass, length, radius)
        return gtd.Link(link_id, link_name, mass, inertia, pose, lTcom, False)

    @staticmethod
    def compute_link_inertia(mass: float, length: float, radius: float):
        """ Compute inertia matrix for a link. """
        Ixx = 1/12 * mass * (3*radius**2 + length**2)
        Iyy = 1/2 * mass * radius**2
        Izz = Ixx
        inertia = np.diag([Ixx, Iyy, Izz])
        return inertia

    @staticmethod
    def icra_init_config():
        """ initial configuraiton used in ICRA paper. """
        q_knee = np.radians(161.7)
        q_hip = np.radians(-59.1)
        q_foot = np.radians(-12.6)

        q1 = -q_hip
        q2 = q1 + np.pi - q_knee
        foot_dist = 2 * 0.55 * (0.5 + np.cos(q1) - np.cos(q2))
        torso_height = 0.55 * (np.sin(q2) - np.sin(q1))
        torso_pose = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, 0, torso_height))
        torso_twist = np.zeros(6)

        init_vels = np.zeros(6)

        angle_offset = np.arcsin((foot_dist-0.55)/2 / 1.1)
        rest_angles = [q_foot - angle_offset,
                    q_knee,
                    q_hip - np.pi/2 + angle_offset,
                    q_hip - np.pi/2 + angle_offset,
                    q_knee,
                    q_foot - angle_offset]
        init_angles = rest_angles

        P_s_0 = 65.0 * 6.89476 + 101.325
        init_config = JumpingRobot.create_init_config(torso_pose, torso_twist,
                                                    rest_angles, init_angles,
                                                    init_vels, P_s_0, foot_dist)
        return init_config

    @staticmethod
    def icra_yaml():
        """ yaml file path specifying parameters used in ICRA. """
        return "examples/example_jumping_robot/yaml/robot_config.yaml"

    @staticmethod
    def simple_init_config():
        """ A simple initial configuration: knees are bent to 60 degrees. """
        theta = np.pi/3
        rest_angles = [-theta, 2 * theta, -theta, -theta, 2*theta, -theta]
        init_angles = rest_angles
        init_vels = [0, 0, 0, 0, 0, 0]
        torso_pose = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, 0, 0.55))
        torso_twist = np.zeros(6)
        P_s_0 = 65 * 6894.76/1000
        foot_dist = 0.55
        init_config = JumpingRobot.create_init_config(torso_pose, torso_twist,
                                                    rest_angles, init_angles,
                                                    init_vels, P_s_0, foot_dist)
        return init_config

    @staticmethod
    def get_camera_calibration(cam_params):
        ''' Set up initial camera calibration '''
        mtx = cam_params['matrix']
        dist = cam_params['dist']
        dim = cam_params['dimension']

        fy = mtx[0][0] # switched x- and y- axes
        fx = mtx[1][1] # switched x- and y- axes
        f = (fx+fy)/2 # (pixels) focal length
        k1 = dist[0] # first radial distortion coefficient (quadratic)
        k2 = dist[1] # second radial distortion coefficient (quartic)
        p1 = dist[2] # first tangential distortion coefficient
        p2 = dist[3] # second tangential distortion coefficient
        k3 = dist[4] # third radial distortion coefficient
        u0 = dim[1]/2 # (pixels) principal point
        v0 = dim[0]/2 # (pixels) principal point
        calibration = gtsam.Cal3Bundler(f, k1, k2, u0, v0)
        return calibration 

    @staticmethod
    def get_cam_params(path_cam_params):
        """ Read camera parameter file. """
        with open(path_cam_params) as file:
            cam_params = yaml.load(file, Loader=yaml.FullLoader)
        return cam_params

    @staticmethod
    def get_marker_locations():
        ''' Get (relative) marker locations from link poses '''
        # locations = [[[-0.071,  -0.1,    0], [-0.071,    0.1,    0]], # switched y/z
        #              [[-0.038,  -0.1,    0], [-0.038,    0.1,    0]], # switched y/z
        #              [[     0,  0.15,    0], [     0,  -0.15,    0]], # updated for 30-cm hip marker spacing
        #              [[-0.038,   0.1,    0], [-0.038,   -0.1,    0]], # switched y/z
        #              [[-0.071,   0.1,    0], [-0.071,   -0.1,    0]]] # switched y/z
        locations = [[[-0.071,   0, -0.1], [-0.071,   0,   0.1]],
                     [[-0.038,   0, -0.1], [-0.038,   0,   0.1]],
                     [[     0, 0.15,   0], [     0, -0.15,   0]],
                     [[-0.038,   0,  0.1], [-0.038,    0, -0.1]],
                     [[-0.071,   0,  0.1], [-0.071,    0, -0.1]]]
        return locations

    @staticmethod
    def CalibrationKey():
        return gtd.DynamicsSymbol.SimpleSymbol('K', 0).key()

    @staticmethod
    def CameraPoseKey():
        return gtd.DynamicsSymbol.SimpleSymbol('cp', 0).key()
    
    @staticmethod
    def MarkerKey (link_idx, marker_idx, k):
        return gtd.DynamicsSymbol.LinkJointSymbol('mk', link_idx, marker_idx, k).key()
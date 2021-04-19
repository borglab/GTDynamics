"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information
Tests to develop the calibration of the jumping robot.
Author: Yetong Zhang
"""
from gtsam.utils.test_case import GtsamTestCase
import os
import gtdynamics as gtd
import gtsam
import numpy as np
import unittest

import os
import sys
import inspect
currentdir = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)

from src.jr_visualizer import visualize_jr
from src.jumping_robot import Actuator, JumpingRobot



# noise model for measurements
model_marker = gtsam.noiseModel.Isotropic.Sigma(3, 0.01)
model_projection = gtsam.noiseModel.Isotropic.Sigma(2, 0.1)
model_cam_pose_prior = gtsam.noiseModel.Isotropic.Sigma(6, 0.1)
model_calib = gtsam.noiseModel.Isotropic.Sigma(3, 1)
# this should be small,since it's like hard constrain
model_kinematics = gtsam.noiseModel.Diagonal.Sigmas(
    [0.01, 0.01, 0.01, 0.01, 0.01, 0.01])


def LinkPoseKey(link_idx, k):
    return gtd.DynamicsSymbol.LinkSymbol('p', link_idx, k).key()


def JointAngleKey(joint_idx, k):
    return gtd.DynamicsSymbol.JointSymbol('q', joint_idx, k).key()


def MarkerKey(link_idx, marker_idx, k):
    return gtd.DynamicsSymbol.LinkJointSymbol('m', link_idx, marker_idx, k).key()


def CalibrationKey():
    return gtd.DynamicsSymbol.SimpleSymbol('k', 0).key()


def CameraPoseKey():
    return gtd.DynamicsSymbol.SimpleSymbol('c', 0).key()


class TestCalibration(GtsamTestCase):

    def load_robot(self):
        yaml_file_path = "examples/example_jumping_robot/yaml/robot_config.yaml"
        init_config = JumpingRobot.create_init_config()
        jr = JumpingRobot(yaml_file_path, init_config, phase=3)
        return jr.robot

    def get_marker_locations(self):
        ''' Get marker locations from link poses '''
        locations = [[[0, 0, -0.125], [0, 0, 0.125]],
                     [[0, 0, -0.125], [0, 0, 0.125]],
                     [[0, 0.125, 0], [0, -0.125, 0]],
                     [[0, 0, 0.125], [0, 0, -0.125]],
                     [[0, 0, 0.125], [0, 0, -0.125]]]
        return locations

    def get_pixel_measurement(self):
        ''' Import marker pixel location measurements '''
        pixels = [[[0.15, 0.275], [0.4, 0.275]],
                  [[0.7, 0.275], [0.95, 0.275]],
                  [[1.1, 0.125], [1.1, -0.125]],
                  [[0.95, -0.275], [0.7, -0.275]],
                  [[0.4, -0.275], [0.15, -0.275]]]
        pixels_all_frames = [pixels]
        return pixels_all_frames

    def get_calibration(self):
        focal_length = 1  # how large projected image is
        distortion1 = 0
        distortion2 = 0
        offset_u = 0
        offset_v = 0
        calibration = gtsam.Cal3Bundler(
            focal_length, distortion1, distortion2, offset_u, offset_v)
        return calibration

    def get_robot_config(self):
        link_poses = [gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, 0.275, 0.275)),
                      gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, 0.275, 0.825)),
                      gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, 0, 1.1)),
                      gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(
                          0, -0.275, 0.825)),
                      gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, -0.275, 0.275))]

        def gauss_rand(sigma):
            return np.random.normal(0, sigma)

        # add noise to link_pose
        for i in range(len(link_poses)):
            pose_noise = gtsam.Pose3(gtsam.Rot3.RzRyRx(gauss_rand(0.1), gauss_rand(0.1), gauss_rand(
                0.1)), gtsam.Point3(gauss_rand(0.1), gauss_rand(0.1), gauss_rand(0.1)))
            link_poses[i] = link_poses[i].transformPoseFrom(pose_noise)

        joint_angles = [
            0 + gauss_rand(0.5), 0 + gauss_rand(0.5), 0 + gauss_rand(0.5), 0 + gauss_rand(0.5)]

        return link_poses, joint_angles

    def test_calibration(self):
        ################################################
        ############ set initial parameters ############
        ################################################
        my_robot = self.load_robot()  # load jumping robot

        marker_locations = self.get_marker_locations()
        pixels_all_frames = self.get_pixel_measurement()
        num_frames = len(pixels_all_frames)

        # camera pose & calibration
        cam_pose = gtsam.Pose3(gtsam.Rot3.Ry(-np.pi/2), gtsam.Point3(1, 0, 0))
        calibration = self.get_calibration()
        link_poses, joint_angles = self.get_robot_config()

        ##########################################
        ############ construct values ############
        ##########################################
        initial_estimate = gtsam.Values()

        for k in range(num_frames):
            for i in range(1, my_robot.numLinks()+1):  # loop over link poses
                key = LinkPoseKey(i, k)
                initial_estimate.insert(key, link_poses[i-1])

            for j in range(1, my_robot.numJoints()+1):  # loop over joints
                key = JointAngleKey(j, k)
                initial_estimate.insertDouble(key, 0)

            for i in range(1, my_robot.numLinks()+1):
                markers_i = marker_locations[i-1]
                for idx_marker in range(len(markers_i)):
                    marker_location_local = np.array(markers_i[idx_marker])
                    link_pose = link_poses[i-1]
                    marker_location = link_pose.transformFrom(
                        marker_location_local)
                    key = MarkerKey(i, idx_marker, k)
                    initial_estimate.insert(key, marker_location)

        cal_key = CalibrationKey()
        initial_estimate.insert(cal_key, calibration)

        cam_pose_key = CameraPoseKey()
        initial_estimate.insert(cam_pose_key, cam_pose)

        ################################################
        ############ construct factor graph ############
        ################################################
        graph = gtsam.NonlinearFactorGraph()

        # add pose factors
        pixels_all_frames = self.get_pixel_measurement()
        graph_builder = gtd.DynamicsGraph()
        for k in range(num_frames):
            graph.push_back(graph_builder.qFactors(my_robot, k, None))

            # add pose to feature point transform factors
            for i in range(1, my_robot.numLinks()+1):
                markers_i = marker_locations[i-1]
                link_pose_key = LinkPoseKey(i, k)
                for idx_marker in range(len(markers_i)):
                    marker_key = MarkerKey(i, idx_marker, k)
                    marker_location = np.array(markers_i[idx_marker])
                    graph.push_back(gtd.PosePointFactor(
                        link_pose_key, marker_key, model_marker, marker_location))

                    pixel_meas = np.array(
                        pixels_all_frames[k][i-1][idx_marker])
                    graph.push_back(gtd.CustomProjectionFactor(
                        pixel_meas, model_projection, cam_pose_key, marker_key, cal_key))

        # prior factor for the camera pose
        graph.add(gtsam.PriorFactorPose3(
            cam_pose_key, cam_pose, model_cam_pose_prior))
        graph.add(gtd.PriorFactorCal3Bundler(
            cal_key, calibration, model_calib))

        ################################################
        ############ solve the factor graph ############
        ################################################
        print("initial error: ", graph.error(initial_estimate))

        params = gtsam.LevenbergMarquardtParams()
        optimizer = gtsam.LevenbergMarquardtOptimizer(
            graph, initial_estimate, params)
        result = optimizer.optimize()

        print("result error: ", graph.error(result))

        ##########################################
        ############ print link poses ############
        ##########################################
        for k in range(num_frames):
            print("Frame {}:".format(k))
            for i in range(1, my_robot.numLinks() + 1):
                key = LinkPoseKey(i, k)
                pose = result.atPose3(key)
                print("Link {}: ".format(i))
                print(pose)

        self.assertEqual(initial_estimate.size(), num_frames*(5+4+10)+2)
        self.assertEqual(graph.size(), num_frames*4 + num_frames *
                         my_robot.numLinks()*2*2 + 1 + 1)  # check number of factors


if __name__ == "__main__":
    unittest.main()

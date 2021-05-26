"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  measurement_graph_builder.py
 * @brief Create measurement factor graphs for the jumping robot system identification.
 * @author Lucas Tziani, Yetong Zhang
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


class MeasurementGraphBuilder:
    """ Class that constructs measurement graph for system identification. """
    def __init__(self):
        ratio = 2
        self.model_cam_pose_prior = gtsam.noiseModel.Isotropic.Sigma(6, 0.05) # (rad, m) 0.05
        self.model_calib = gtsam.noiseModel.Isotropic.Sigma(3, 1) # (pix) focal length & offsets 1
        self.model_marker = gtsam.noiseModel.Isotropic.Sigma(3, 0.01 * ratio) # (m) 0.01
        self.model_projection = gtsam.noiseModel.Isotropic.Sigma(2, 4 * ratio) # (pixels) maybe increase
        self.pressure_meas_model = gtsam.noiseModel.Isotropic.Sigma(1, 10 * ratio)


    def step_pixel_meas_graph(self, jr, k, pixel_frame):
        """ Creates pixel measurement graph for the kth step. """
        graph = NonlinearFactorGraph()

        for link in jr.robot.links():
            if link.name() == "ground":
                continue
            i = link.id()
            markers_i = jr.marker_locations[i-1]
            link_pose_key = gtd.internal.PoseKey(i, k).key()
           
            for idx_marker in range(len(markers_i)):
                marker_key = JumpingRobot.MarkerKey(i, idx_marker, k)
                marker_location = np.array(markers_i[idx_marker])
                graph.push_back(gtd.PosePointFactor(
                    link_pose_key, marker_key, self.model_marker, marker_location))

                cam_pose_key = JumpingRobot.CameraPoseKey()
                cal_key = JumpingRobot.CalibrationKey()
                pixel_meas = pixel_frame[i-1][idx_marker]
                graph.push_back(gtd.CustomProjectionFactor(
                    pixel_meas, self.model_projection, cam_pose_key, marker_key, cal_key)) 
        return graph


    def step_pressure_meas_graph(self, jr, k, step_pressure_measures):
        """ Create pressure measurement factors of a time step. """
        graph = NonlinearFactorGraph()

        for actuator in jr.actuators:
            j = actuator.j
            pressure_key = Actuator.PressureKey(j, k)
            pressure = step_pressure_measures[j]
            graph.add(gtd.PriorFactorDouble(pressure_key, pressure, self.pressure_meas_model))
        
        source_pressure_key = Actuator.SourcePressureKey(k)
        source_pressure = step_pressure_measures[0]
        graph.add(gtd.PriorFactorDouble(source_pressure_key, source_pressure, self.pressure_meas_model))
        return graph


    def camera_priors(self, jr):
        ''' Set up camera factors '''
        graph = NonlinearFactorGraph()

        cam_params = jr.params['cam_params']
        cam_pose_key = JumpingRobot.CameraPoseKey()
        # cam_pose = gtsam.Pose3(gtsam.Rot3.Ry(cam_params['pose']['Ry']), 
        #     np.array(cam_params['pose']['point'])) # camera pose in world frame
        cam_pose = gtsam.Pose3(gtsam.Rot3(cam_params['pose']['R']), 
            np.array(cam_params['pose']['t'])) # camera pose in world frame
        graph.add(gtsam.PriorFactorPose3(cam_pose_key, cam_pose, self.model_cam_pose_prior))  

        cal_key = JumpingRobot.CalibrationKey()
        calibration = JumpingRobot.get_camera_calibration(cam_params)
        gtd.addPriorFactorCal3Bundler(graph, cal_key, calibration, self.model_calib)
        return graph


    def measurement_graph(self, jr, pixels_all_frames, pressures_all_frames):
        """ System identification measurement factors for the trajectory. """
        graph = NonlinearFactorGraph()
        for k in range(len(pressures_all_frames)):
            pixel_meas = pixels_all_frames[k]
            pressure_meas = pressures_all_frames[k]
            graph.push_back(self.step_pixel_meas_graph(jr, k, pixel_meas))
            graph.push_back(self.step_pressure_meas_graph(jr, k, pressure_meas))
        return graph



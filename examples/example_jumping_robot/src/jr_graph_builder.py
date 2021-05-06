"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  jr_graph_builder.py
 * @brief Create factor graphs for the jumping robot.
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
from actuation_graph_builder import ActuationGraphBuilder
from robot_graph_builder import RobotGraphBuilder
from jr_values import JRValues



class JRGraphBuilder:
    """ Class that constructs factor graphs for a jumping robot. """

    def __init__(self):
        """Initialize the graph builder, specify all noise models."""
        self.robot_graph_builder = RobotGraphBuilder()
        self.actuation_graph_builder = ActuationGraphBuilder()
        self.model_marker = gtsam.noiseModel.Isotropic.Sigma(3, 0.01) # (m) 0.01
        self.model_projection = gtsam.noiseModel.Isotropic.Sigma(2, 4) # (pixels) maybe increase
        self.model_cam_pose_prior = gtsam.noiseModel.Isotropic.Sigma(6, 0.05) # (rad, m)
        self.model_calib = gtsam.noiseModel.Isotropic.Sigma(3, 1) # (pix) focal length & offsets 100
        self.pressure_meas_model = gtsam.noiseModel.Isotropic.Sigma(1, 10)

    def collocation_graph(self, jr: JumpingRobot, step_phases: list, collocation):
        """ Create a factor graph containing collocation constraints. """
        graph = self.actuation_graph_builder.collocation_graph(jr, step_phases, collocation)
        graph.push_back(self.robot_graph_builder.collocation_graph(jr, step_phases, collocation))

        # add collocation factors for time
        for time_step in range(len(step_phases)):
            phase = step_phases[time_step]
            k_prev = time_step
            k_curr = time_step+1
            dt_key = gtd.PhaseKey(phase).key()
            time_prev_key = gtd.TimeKey(k_prev).key()
            time_curr_key = gtd.TimeKey(k_curr).key()
            time_col_cost_model = self.robot_graph_builder.graph_builder.opt().time_cost_model
            gtd.AddTimeCollocationFactor(graph, time_prev_key, time_curr_key,
                                         dt_key, time_col_cost_model)

        return graph

    def dynamics_graph(self, jr: JumpingRobot, k: int) -> NonlinearFactorGraph:
        """ Create a factor graph containing dynamcis constraints for 
            the robot, actuators and source tank at a certain time step.
        """
        graph = self.actuation_graph_builder.dynamics_graph(jr, k)
        graph.push_back(self.robot_graph_builder.dynamics_graph(jr, k))
        return graph

    def transition_dynamics_graph(self, prev_jr, new_jr, k):
        """ Dynamics graph for transition node. """
        graph = self.actuation_graph_builder.dynamics_graph(prev_jr, k)
        graph.push_back(self.robot_graph_builder.transition_dynamics_graph(prev_jr, new_jr, k))
        return graph

    def control_priors_actuator(self, jr, actuator, controls):
        """ Create prior factors for control variables of an actuator. """
        graph = NonlinearFactorGraph()
        j = actuator.j
        name = actuator.name
        prior_time_cost_model = self.actuation_graph_builder.prior_time_cost_model
        To_key = Actuator.ValveOpenTimeKey(j)
        Tc_key = Actuator.ValveCloseTimeKey(j)
        graph.add(gtd.PriorFactorDouble(To_key, controls["Tos"][name], prior_time_cost_model))
        graph.add(gtd.PriorFactorDouble(Tc_key, controls["Tcs"][name], prior_time_cost_model))
        return graph

    def control_priors(self, jr, controls):
        """ Create prior factors for control variables (To, Tc). """
        graph = NonlinearFactorGraph()
        for actuator in jr.actuators:
            graph.push_back(self.control_priors_actuator(jr, actuator, controls))
        return graph
    
    def vertical_jump_goal_factors(self, jr, k):
        """ Add goal factor for vertical jumps, at step k. 
            The twist of torso reduces to 0.
        """
        graph = NonlinearFactorGraph()
        torso_i = jr.robot.link("torso").id()
        torso_twist_key = gtd.internal.TwistKey(torso_i, k).key()
        target_twist = np.zeros(6)
        bv_cost_model = self.robot_graph_builder.graph_builder.opt().bv_cost_model
        graph.add(gtd.PriorFactorVector6(torso_twist_key, target_twist, bv_cost_model))
        return graph

    def time_prior(self):
        graph = NonlinearFactorGraph()
        t0_key = gtd.TimeKey(0).key()
        # t0 = init_config_values.atDouble(t0_key)
        time_cost_model = self.robot_graph_builder.graph_builder.opt().time_cost_model
        graph.add(gtd.PriorFactorDouble(t0_key, 0.0, time_cost_model))
        return graph

    def trajectory_priors(self, jr):
        graph = NonlinearFactorGraph()
        init_config_values = JRValues.init_config_values(jr)
        graph.push_back(self.robot_graph_builder.prior_graph(jr, init_config_values, 0))
        graph.push_back(self.actuation_graph_builder.prior_graph(jr, init_config_values, 0))
        graph.push_back(self.time_prior())
        return graph

    def trajectory_graph(self, jr, step_phases, collocation=gtd.CollocationScheme.Euler):
        """ Create a factor graph consisting of all factors represeting
            the robot trajectory.
        """
        # prior factors for init configuration, control and time
        graph = self.trajectory_priors(jr)

        # dynamics graph at each step
        if len(step_phases) == 0:
            graph.push_back(self.dynamics_graph(jr, 0))
            return graph

        jr = jr.jr_with_phase(step_phases[0])
        for k in range(len(step_phases)+1):
            prev_phase = step_phases[k-1] if k!=0 else step_phases[0]
            next_phase = step_phases[k] if k!=len(step_phases) else step_phases[-1]
            if next_phase == prev_phase:
                print(k, prev_phase)
                graph_dynamics = self.dynamics_graph(jr, k)
            else:
                print(k, prev_phase, '->', next_phase)
                new_jr = jr.jr_with_phase(next_phase)
                graph_dynamics = self.transition_dynamics_graph(jr, new_jr, k)
                jr = new_jr
            graph.push_back(graph_dynamics)

        # collocation factors across steps
        graph_collo = self.collocation_graph(jr, step_phases, collocation)
        graph.push_back(graph_collo)

        return graph



    def step_pixel_meas_graph(self, jr, k, marker_locations, pixel_frame):
        """ Creates pixel measurement graph for the kth step. """
        graph = NonlinearFactorGraph()

        for link in jr.robot.links():
            if link.name() == "ground":
                continue
            i = link.id()
            markers_i = marker_locations[i-1]
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


    def get_camera_calibration(self, cam_params):
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


    def camera_priors(self, cam_params):
        ''' Set up camera factors '''
        graph = NonlinearFactorGraph()

        cam_pose_key = JumpingRobot.CameraPoseKey()
        cam_pose = gtsam.Pose3(gtsam.Rot3.Ry(cam_params['pose']['Ry']), 
            gtsam.Point3(cam_params['point'][0], cam_params['point'][1], cam_params['point'][2])) # camera pose in world frame
        graph.add(gtsam.PriorFactorPose3(cam_pose_key, cam_pose, self.model_cam_pose_prior))  

        cal_key = JumpingRobot.CalibrationKey()
        calibration = self.get_camera_calibration(cam_params)
        gtdynamics.addPriorFactorCal3Bundler(graph, cal_key, calibration, self.model_calib)
        return graph


    def sys_id_graph(self, jr, marker_locations, pixels_all_frames, pressures_all_frames):
        """ System identification factors for the trajectory. """
        # set up camera prior factors
        graph = self.camera_priors(path_cam_params)

        # build graph
        for k in range(len(pressures_all_frames)):
            pixel_meas = pixels_all_frames[k]
            pressure_meas = pressures_all_frames[k]
            graph.push_back(self.step_pixel_meas_graph(jr, k, marker_locations, pixel_meas))
            print(graph.size())
            graph.push_back(self.step_pressure_meas_graph(jr, k, pressure_meas))
            print(graph.size())
        return graph


    def sys_id_estimates(self, jr, initial_estimate, marker_locations, 
        pixels_all_frames, pressures_all_frames, cam_params):
        ''' Set initial estimates for system ID '''
        for k in range(num_frames):
            pixel_meas = pixels_all_frames[k]
            pressure_meas = pressures_all_frames[k]

            # add pressures
            for actuator in jr.actuators: 
                j = actuator.j # TODO: is this indexed at 1?
                pressure_key = Actuator.PressureKey(j, k)
                pressure = pressure_meas[j]
                initial_estimate.insert(pressure_key, pressure)

            source_pressure_key = Actuator.SourcePressureKey(k)
            source_pressure = pressure_meas[0]
            initial_estimate.insert(source_pressure_key, source_pressure)


            # add markers
            for link in jr.robot.links():
                if link.name() == "ground":
                    continue
                i = link.id()
                markers_i = marker_locations[i-1]
                for idx_marker in range(len(markers_i)):
                    marker_key = JumpingRobot.MarkerKey(i, idx_marker, k)
                    marker_location = np.array(markers_i[idx_marker])
                    initial_estimate.insert(marker_key, marker_location)

        # add camera calibration
        cal_key = CalibrationKey()
        calibration = self.get_camera_calibration(cam_params)
        initial_estimate.insert(cal_key, calibration) 

        # add camera pose
        cam_pose_key = CameraPoseKey()
        cam_pose = gtsam.Pose3(gtsam.Rot3.Ry(cam_params['pose']['Ry']), 
            gtsam.Point3(cam_params['point'][0], cam_params['point'][1], cam_params['point'][2])) 
        initial_estimate.insert(cam_pose_key, cam_pose)

        return initial_estimate
    
            




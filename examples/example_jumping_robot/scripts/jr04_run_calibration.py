"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information
Tests to develop the calibration of the jumping robot.
Author: Yetong Zhang
"""

import numpy as np
import gtsam
import gtdynamics

import os
import sys
import inspect
currentdir = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)

from src.jr_visualizer import visualize_jr
from src.jumping_robot import Actuator, JumpingRobot

import matplotlib.pyplot as plt


# paths
path_data = 'examples/example_jumping_robot/system-id-data'
prefix_data = '0p00_0p12_hipknee-80source 2021-04-05 11-43-47'

# camera
# dim_camera = [720, 1280] # iPhone is 720 x 1280 (oriented vertically)
dim_camera = [1920, 1080]    # GoPro is 1920 x 1080 (oriented horizontally)

# noise model for measurements TODO: change noise models?
model_marker = gtsam.noiseModel.Isotropic.Sigma(3, 0.01) # (m) 0.01
model_projection = gtsam.noiseModel.Isotropic.Sigma(2, 4) # (pixels) maybe increase
model_cam_pose_prior = gtsam.noiseModel.Isotropic.Sigma(6, 0.05) # (rad, m)
model_calib = gtsam.noiseModel.Isotropic.Sigma(3, 1) # (pix) focal length & offsets 100
model_kinematics = gtsam.noiseModel.Diagonal.Sigmas([0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001]) # (rad, m) this should be small,since it's like hard constraint 0.001


def LinkPoseKey(link_idx, k):
    return gtdynamics.DynamicsSymbol.LinkSymbol('p',link_idx,k).key()

def JointAngleKey (joint_idx, k):
    return gtdynamics.DynamicsSymbol.JointSymbol('q',joint_idx,k).key()

def MarkerKey (link_idx, marker_idx, k):
    return gtdynamics.DynamicsSymbol.LinkJointSymbol('m', link_idx, marker_idx, k).key()

def CalibrationKey():
    return gtdynamics.DynamicsSymbol.SimpleSymbol('K', 0).key()

def CameraPoseKey():
    return gtdynamics.DynamicsSymbol.SimpleSymbol('c', 0).key()


current_folder = os.path.dirname(os.path.abspath(__file__))

class TestCalibration():
    def __init__(self, path_data, prefix_data):
        self.path_data = path_data
        self.prefix_data = prefix_data


    def load_robot(self):
        yaml_file_path = "examples/example_jumping_robot/yaml/robot_config_2021-04-05.yaml"
        init_config = JumpingRobot.create_init_config()
        jr = JumpingRobot.from_yaml(yaml_file_path, init_config, phase=3)
        return jr.robot


    def get_pixel_measurement(self): 
        ''' Import marker pixel location measurements ''' 
        file_marker_pix = os.path.join(self.path_data, self.prefix_data, (self.prefix_data + ' marker_pix.txt'))
        marker_pix = np.loadtxt(file_marker_pix, dtype='float', comments='#', delimiter=',')
        marker_pix[:,10:] = dim_camera[1] - marker_pix[:,10:] # flip y-axis
        marker_pix = np.fliplr(marker_pix) # swap x & y axes, reverse marker numbering

        pixels = np.zeros([len(marker_pix), 5, 2, 2]) # [frame,link,marker,x/y pix]
        for k in range(len(marker_pix)): # loop over frames
            for j in range(5): # loop over links
                for i in range(2): # loop over markers on link
                    pixels[k,j,i] = [marker_pix[k,j*2+i], marker_pix[k,j*2+i+10]] 
        return pixels


    def get_calibration(self):
        ''' Set up initial camera calibration '''
        mtx = [[930.61550934,   0.,             957.52437616], # from OpenCV checkerboard cal
               [0,              932.40029846,   552.74942762],
               [0,              0,              1           ]] 
        dist = [-0.271825734,  0.0864954022, -0.000255924635,  0.000889888021, -0.0136882404]

        fy = mtx[0][0] # switched x- and y- axes
        fx = mtx[1][1] # switched x- and y- axes
        f = (fx+fy)/2 # (pixels) focal length
        k1 = dist[0] # first radial distortion coefficient (quadratic)
        k2 = dist[1] # second radial distortion coefficient (quartic)
        p1 = dist[2] # first tangential distortion coefficient
        p2 = dist[3] # second tangential distortion coefficient
        k3 = dist[4] # third radial distortion coefficient
        u0 = dim_camera[1]/2 # (pixels) principal point
        v0 = dim_camera[0]/2 # (pixels) principal point
        calibration = gtsam.Cal3Bundler(f, k1, k2, u0, v0)
        return calibration


    def get_robot_config(self):
        ''' Load robot pose data from preprocessed results '''
        file_pose = os.path.join(self.path_data, self.prefix_data, (self.prefix_data + ' pose_pre.txt'))
        pose_pre = np.loadtxt(file_pose, dtype='float', comments='#', delimiter=',')

        n_frames = int(np.shape(pose_pre)[0]/5)
        pose_pre = np.reshape(pose_pre, (5,n_frames,4,4)) # unflatten
        link_poses = [None]*n_frames
        for k in range(n_frames): # loop over frames
            pose = []
            for j in range(5): # loop over links
                p = pose_pre[j,k,:,:]
                p[1,3] = p[1,3] - 0.0254 # shift to world frame
                p[2,3] = p[2,3] - 1.07315 # shift to world frame
                p_old = gtsam.Pose3(p)
                # if j == 2: # keep torso pose the same
                #     T_old_new = gtsam.Pose3()
                # else: # rotate other link poses by 90
                #     T_old_new = gtsam.Pose3(gtsam.Rot3.Rx(np.pi/2), gtsam.Point3(0, 0, 0))
                # p_new = p_old.compose(T_old_new)
                pose.append(p_old)
            link_poses[k] = pose

        file_angle = os.path.join(self.path_data, self.prefix_data, (self.prefix_data + ' angle_pre.txt'))
        joint_angles = np.loadtxt(file_angle, dtype='float', comments='#', delimiter=',')
        return link_poses, joint_angles


    def test_calibration(self):
        ################################################
        ############ set initial parameters ############
        ################################################
        my_robot = self.load_robot()  # load jumping robot

        # fiducial markers
        marker_locations = JumpingRobot.get_marker_locations() # marker locations
        pixels_all_frames = self.get_pixel_measurement() # marker pixel measurments
        num_frames = len(pixels_all_frames)

        # camera pose & calibration
        cam_pose = gtsam.Pose3(gtsam.Rot3.Ry(-np.pi/2), gtsam.Point3(2.5019, 0, 0)) # camera pose in world frame
        # cam_pose = gtsam.Pose3(gtsam.Rot3(cam_params['pose']['R']), 
        #     np.array(cam_params['pose']['t'])) # camera pose in world frame
        calibration = self.get_calibration()
        link_poses, joint_angles = self.get_robot_config()

        
        ##########################################
        ############ construct values ############
        ##########################################
        initial_estimate = gtsam.Values()

        for k in range(num_frames):
            for i in range(1, my_robot.numLinks()+1): # loop over link poses
                key = LinkPoseKey(i, k)
                initial_estimate.insert(key, link_poses[k][i-1])

            for j in range(1, my_robot.numJoints()+1): # loop over joints
                key = JointAngleKey(j, k)
                initial_estimate.insertDouble(key, joint_angles[k][j-1])

            for i in range(1, my_robot.numLinks()+1): # loop over markers
                markers_i = marker_locations[i-1]
                for idx_marker in range(len(markers_i)):
                    marker_location_local = np.array(markers_i[idx_marker])
                    link_pose = link_poses[k][i-1]
                    marker_location = link_pose.transformFrom(marker_location_local)
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
        graph_builder = gtdynamics.DynamicsGraph()
        for k in range(num_frames):
            kin_factor_graph = graph_builder.qFactors(my_robot, k, None)
            graph.push_back(kin_factor_graph)

            # add pose to feature point transform factors
            for i in range(1, my_robot.numLinks()+1):
                markers_i = marker_locations[i-1]
                link_pose_key = LinkPoseKey(i, k)
                for idx_marker in range(len(markers_i)):
                    marker_key = MarkerKey(i, idx_marker, k)
                    marker_location = np.array(markers_i[idx_marker])
                    graph.push_back(gtdynamics.PosePointFactor(
                        link_pose_key, marker_key, model_marker, marker_location))

                    pixel_meas = pixels_all_frames[k][i-1][idx_marker]
                    graph.push_back(gtdynamics.CustomProjectionFactor(
                        pixel_meas, model_projection, cam_pose_key, marker_key, cal_key)) 

        # prior factor for the camera pose
        graph.add(gtsam.PriorFactorPose3(cam_pose_key, cam_pose, model_cam_pose_prior))        
        gtdynamics.addPriorFactorCal3Bundler(graph, cal_key, calibration, model_calib)


        ################################################
        ############ solve the factor graph ############
        ################################################
        print("initial error: ", graph.error(initial_estimate))

        params = gtsam.LevenbergMarquardtParams()
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate, params)
        result = optimizer.optimize()
        
        print("result error: ", graph.error(result))


        ##########################################
        ############ print results ###############
        ##########################################
        # for k in range(num_frames):
        #     print("Frame {}:".format(k))
        #     for i in range(1, my_robot.numLinks() + 1):
        #         key = LinkPoseKey(i, k)
        #         pose = result.atPose3(key)
        #         print("Link {}: ".format(i))
        #         print(pose)

        #     for j in range(1, my_robot.numJoints()+1): # loop over joints
        #         key = JointAngleKey(j, k)
        #         angle = result.atDouble(key)
        #         print("Joint {}: ".format(j))
        #         print(angle)

        #     for i in range(1, my_robot.numLinks()+1):
        #         markers_i = marker_locations[i-1]
        #         print("Link {}: ".format(i))
        #         for idx_marker in range(len(markers_i)):
        #             key = MarkerKey(i, idx_marker, k)
        #             marker = result.atVector(key)
        #             print("Marker {}: ".format(idx_marker))
        #             print(marker)
            
        print(result.atPose3(cam_pose_key))
        print(result.atCal3Bundler(cal_key))


        # export pose data
        pose_data = np.zeros([my_robot.numLinks(),num_frames,16])
        for k in range(num_frames):
            for i in range(1, my_robot.numLinks() + 1):
                key = LinkPoseKey(i, k)
                pose = result.atPose3(key)
                pose_data[i-1,k,:] = pose.matrix().flatten()
        pose_data = np.reshape(pose_data, [num_frames*my_robot.numLinks(), 16]) # all pose 1, then all pose 2, etc
         
        # file_pose = os.path.join(self.path_data, self.prefix_data, 
        #     (self.prefix_data + ' pose_gtsam.txt'))
        # np.savetxt(file_pose, pose_data, fmt='%.6e', delimiter=',')


        # export angle data
        angle_data = np.zeros([num_frames,my_robot.numJoints()])
        for k in range(num_frames):
            for j in range(1, my_robot.numJoints()+1): # loop over joints
                key = JointAngleKey(j, k)
                angle_data[k,j-1] = result.atDouble(key)
                
        # file_angle = os.path.join(self.path_data, self.prefix_data, 
        #     (self.prefix_data + ' angle_gtsam.txt'))
        # np.savetxt(file_angle, angle_data, fmt='%.6e', delimiter=',')

        # plot 
        x_vec = np.arange(0, np.shape(angle_data)[0])
        fig = plt.figure()
        ax = fig.add_subplot(1,1,1)
        ax.plot(x_vec, np.rad2deg(angle_data[:,0]), 'r')
        ax.plot(x_vec, np.rad2deg(joint_angles[:,0]), 'r--')
        ax.plot(x_vec, np.rad2deg(angle_data[:,1]), 'g')
        ax.plot(x_vec, np.rad2deg(joint_angles[:,1]), 'g--')
        ax.plot(x_vec, np.rad2deg(angle_data[:,2]), 'b')
        ax.plot(x_vec, np.rad2deg(joint_angles[:,2]), 'b--')
        ax.plot(x_vec, np.rad2deg(angle_data[:,3]), 'k')
        ax.plot(x_vec, np.rad2deg(joint_angles[:,3]), 'k--')
        plt.grid()
        plt.show()
        


if __name__ == "__main__":
    tc = TestCalibration(path_data, prefix_data)
    tc.test_calibration()


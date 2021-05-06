"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  helpers.py
 * @brief Helper functions.
 * @author Yetong Zhang
"""

import gtsam
import numpy as np
import os


def mergeValues(values: gtsam.Values, values_add: gtsam.Values, overwrite=False):
    """ insert values, skip duplicate keys

    Args:
        values (gtsam.Values): values to insert into
        values_add (gtsam.Values): values to insert
        overwrite (bool): overwrite values if duplicate keys
    """
    new_values = gtsam.Values(values_add)
    for key in values_add.keys():
        if values.exists(key):
            if overwrite:
                values.erase(key)
            else:
                new_values.erase(key)
    values.insert(new_values)


def read_t_valve(path_data: str) -> np.array:
    ''' Import valve open/close times '''
    prefix_data = os.path.basename(os.path.normpath(path_data))
    file_t_musc = os.path.join(path_data, (prefix_data + ' t_valve.txt'))
    t_valve = np.loadtxt(file_t_musc, dtype='float', comments='#', delimiter=',')
    return t_valve


def read_pressure(path_data: str) -> (np.array, np.array):
    ''' Import experimental pressure data '''
    prefix_data = os.path.basename(os.path.normpath(path_data))
    file_pressures = os.path.join(path_data, (prefix_data + ' pres_filt.txt'))
    data = np.loadtxt(file_pressures, dtype='float', comments='#', delimiter=',')
    time = data[:,0]
    pressure = data[:,1:]
    return time, pressure


def read_marker_pix(path_data: str, dim_camera) -> (np.array, np.array):
    ''' Import marker pixel location measurements ''' 
    prefix_data = os.path.basename(os.path.normpath(path_data))
    file_vid_time = os.path.join(path_data, (prefix_data + ' vid_time.txt'))
    time = np.loadtxt(file_vid_time, dtype='float', comments='#', delimiter=',')
    time = time - time[0] # zero start time

    file_marker_pix = os.path.join(path_data, (prefix_data + ' marker_pix.txt'))
    marker_pix = np.loadtxt(file_marker_pix, dtype='float', comments='#', delimiter=',')
    marker_pix[:,10:] = dim_camera[1] - marker_pix[:,10:] # flip y-axis
    marker_pix = np.fliplr(marker_pix) # swap x & y axes, reverse marker numbering

    pix = np.zeros([len(marker_pix), 5, 2, 2]) # [frame,link,marker,x/y pix]
    for k in range(len(marker_pix)): # loop over frames
        for j in range(5): # loop over links
            for i in range(2): # loop over markers on link
                pix[k,j,i] = [marker_pix[k,j*2+i], marker_pix[k,j*2+i+10]] 
    return time, pix


def interp_pressure(time: np.array, pressure: np.array, time_interp: np.array) -> np.array:
    ''' Interpolate pressures from time array '''
    pressure_interp = np.zeros([time_interp.shape[0], pressure.shape[1]])
    for i in range(pressure.shape[1]):
        pressure_interp[:,i] = np.interp(time_interp, time, pressure[:,i])
    return pressure_interp


def interp_marker_pix(time: np.array, marker_pix: np.array, time_interp: np.array) -> np.array:
    marker_pix_interp = np.zeros([time_interp.shape[0], marker_pix.shape[1],
        marker_pix.shape[2], marker_pix.shape[3]])

    for i in range(5): # loop over links
        for j in range(2): # loop over markers on link
            marker_pix_interp[:,i,j,0] = np.interp(time_interp, time, marker_pix[:,i,j,0]) # u
            marker_pix_interp[:,i,j,1] = np.interp(time_interp, time, marker_pix[:,i,j,1]) # v

    return marker_pix_interp


# # use
# path_data = '/home/cs3630/Documents/system-id-data/0p00_0p12_hipknee-80source 2021-04-05 11-43-47'
# dim_camera = [1920, 1080]    # GoPro is 1920 x 1080 (oriented horizontally)
# time_interp = np.arange(0,0.5,0.01)

# time_exp, pressure_exp = read_pressure(path_data)
# time_vid, pix_vid = read_marker_pix(path_data, dim_camera)

# pressure_interp = interp_pressure(time_exp, pressure_exp, time_interp)
# pix_interp = interp_marker_pix(time_vid, pix_vid, time_interp)

# # plot
# import matplotlib.pyplot as plt

# # plot interpolated markers
# fig = plt.figure()
# for i in range(5):
#     ax = fig.add_subplot(1,5,i+1)
#     ax.plot(time_vid, pix_vid[:,i,0,0], 'r')
#     ax.plot(time_vid, pix_vid[:,i,0,1], 'g')
#     ax.plot(time_vid, pix_vid[:,i,1,0], 'b')
#     ax.plot(time_vid, pix_vid[:,i,1,1], 'k')
#     ax.plot(time_interp, pix_interp[:,i,0,0], 'r--')
#     ax.plot(time_interp, pix_interp[:,i,0,1], 'g--')
#     ax.plot(time_interp, pix_interp[:,i,1,0], 'b--')
#     ax.plot(time_interp, pix_interp[:,i,1,1], 'k--')
# plt.grid()
# plt.show()


# # plot interpolated pressures
# fig = plt.figure()
# ax = fig.add_subplot(1,1,1)
# ax.plot(time_exp, np.rad2deg(pressure_exp[:,0]), 'r')
# ax.plot(time_interp, np.rad2deg(pressure_interp[:,0]), 'r--')
# ax.plot(time_exp, np.rad2deg(pressure_exp[:,1]), 'g')
# ax.plot(time_interp, np.rad2deg(pressure_interp[:,1]), 'g--')
# ax.plot(time_exp, np.rad2deg(pressure_exp[:,2]), 'b')
# ax.plot(time_interp, np.rad2deg(pressure_interp[:,2]), 'b--')
# ax.plot(time_exp, np.rad2deg(pressure_exp[:,3]), 'k')
# ax.plot(time_interp, np.rad2deg(pressure_interp[:,3]), 'k--')
# ax.plot(time_exp, np.rad2deg(pressure_exp[:,4]), 'm')
# ax.plot(time_interp, np.rad2deg(pressure_interp[:,4]), 'm--')
# plt.grid()
# plt.show()
        
"""
GTDynamics Copyright 2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

@file  draw_cdpr.py
@brief Utility functions for drawing a cable robot's controller using opencv
@author Frank Dellaert
@author Gerry Chen
"""

import gtdynamics as gtd
import gtsam
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import cv2
from gtsam import Pose3, Rot3

from cdpr_planar import Cdpr
from cdpr_controller_ilqr import CdprControllerIlqr
from draw_cdpr import pose32xy, a_coords, b_coords, ab_coords

COLORS = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (127, 127, 127)]

def x2img(x):
    return x * 130 + 5
def img2x(img):
    return (img - 5) / 130
def condition(pts):
    """Rescales and reshapes a 2xN numpy matrix to be passed into cv2.polylines
    """
    return x2img(pts.T.reshape((-1, 1, 2))).astype(np.int32)

def draw_cdpr(img, cdpr, x):
    """Draws the CDPR in the specified axis.
    Args:
        ax (plt.Axes): matplotlib axis object
        cdpr (Cdpr): cable robot object
        x (gtsam.Pose3): current pose of the end effector
    
    Returns:
        tuple(): matplotlib line objects:
            l_a - the frame
            l_b - the end effector
            ls_ab - a 4-list containing the 4 cable lines
    """
    cv2.polylines(img, [condition(a_coords(cdpr))], False, (0, 0, 0))
    cv2.polylines(img, [condition(b_coords(cdpr, x))], False, (202, 164, 114))
    for ji in range(4):
        cv2.polylines(img, [condition(ab_coords(cdpr, x, ji))], False, COLORS[ji])
    # ax.set_xlabel('x(m)');ax.set_ylabel('y(m)');ax.set_title('Trajectory');ax.grid()
    return img

def draw_traj(img, cdpr, controller, act_xy, N):
    """Draws the desired and actual x/y trajectories"""
    img = img.astype(np.float)
    kw2 = 81
    kw = 40
    for k in range(N):
        K, uff, Vff, Tff = controller.gains_ff[k]
        K = np.abs(K) / 400 * 7
        # y, x = act_xy[k, 0, :]
        y, x = x2img(Tff.translation()[[0, 2]]).astype(np.int)
        for ci in range(4):
            patch = np.zeros((kw2, kw2, 1))
            patch[kw, kw] = 1
            patch[:, :, 0] = cv2.GaussianBlur(patch, (kw2, kw2), sigmaX=K[ci, 9], sigmaY=K[ci, 11]) * (4-ci)
            alph_src = img[x - kw:x + kw + 1, y - kw:y + kw + 1, 3:]
            img_src = img[x - kw:x + kw + 1, y - kw:y + kw + 1, :3]
            alph_dst = alph_src * (1 - patch) + patch
            tmp = (img_src * alph_src)
            img[x - kw:x + kw + 1, y - kw:y + kw + 1, :3] = (
                img_src * alph_src * (1 - patch) + COLORS[ci] * patch) / alph_dst
            img[x - kw:x + kw + 1, y - kw:y + kw + 1, 3:] = alph_dst
    img[:, :, 3] = img[:, :, 3]*255
    img = img.astype(np.uint8)
    imgtraj = cv2.polylines(np.zeros(img.shape[:2]), [act_xy], False, (1))
    img[imgtraj!=0, :3] = 0
    return img

def draw_controller_one(img, cdpr, controller: CdprControllerIlqr, k, cablei):
    """Draws the controller torque response for every point x in the space, at time step k"""
    xvals = img2x(np.arange(0, img.shape[0]))
    yvals = img2x(np.arange(0, img.shape[1]))
    K, uff, Vff, Tff = controller.gains_ff[k]
    K = K * 100  # this needs to get scaled with the square of dN
    dx = np.zeros((12, *img.shape[:-1]))
    dx[9, :] = (xvals - Tff.translation()[0]).reshape((-1, 1))
    dx[11, :] = (yvals - Tff.translation()[2]).reshape((1, -1))
    uvals = np.einsum('i,ijk->jk', K[cablei, :], dx) + uff[cablei]
    uvals = (uvals + 500) / 1000
    for ci in range(3):
        img[:, :, ci] = np.uint8(uvals * COLORS[cablei][ci])
    return img


def draw_controller_anim(cdpr: Cdpr,
                         controller: CdprControllerIlqr,
                         result: gtsam.Values,
                         N,
                         step=1):
    """Animates the cdpr and feedback gains in side-by-side subplots.

    Args:
        cdpr (Cdpr): cable robot object
        controller (CdprControllerIlqr): cable robot controller object
        result (gtsam.Values): the data from the simulation including poses and tensions
        N (int): number of discrete samples
        step (int, optional): number of time steps to update the animation by each time. 1 doesn't skip any frames, and e.g. 10 would skip 9 frames at a time and update at a period of 10*dt. Defaults to 1.

    Returns:
        matplotlib.animation.FuncAnimation: a matplotlib animation object
    """
    # extract useful variables as lists
    act_T = [gtd.Pose(result, cdpr.ee_id(), k) for k in range(N+1)]
    act_xy = np.array([pose32xy(pose) for pose in act_T]).T
    w, h = np.max(cdpr.params.a_locs, axis=0)[[0, 2]]

    # plot
    def get_imgs(k):
        if k == 0:
            k = N - 20
        img_cdpr = 255 * np.ones((int(x2img(h)+5), int(x2img(w)+5), 3), dtype=np.uint8)
        cv2.polylines(img_cdpr, [condition(act_xy[:, :k])], False, (0, 0, 0))
        draw_cdpr(img_cdpr, cdpr, act_T[k])
        img_cdpr = img_cdpr
        imgs = []
        for i in range(4):
            img = np.zeros((int(x2img(h)+5), int(x2img(w)+5), 3), dtype=np.uint8)
            draw_controller_one(img, cdpr, controller, k, i)
            cv2.polylines(img_cdpr, [condition(act_xy[:, :k])], False, (0, 0, 0))
            draw_cdpr(img, cdpr, act_T[k])
            imgs.append(img)
        img_traj = 255 * np.ones((int(x2img(h)+5), int(x2img(w)+5), 4), dtype=np.uint8)
        img_traj[:, :, 3] = 1
        img_traj = draw_traj(img_traj, cdpr, controller, condition(act_xy).astype(np.int), N)
        return img_cdpr, imgs, img_traj

    fig, axes = plt.subplots(1, 5, figsize=(18, 3))
    def plot_imgs(axes, k):
        img, imgs, _ = get_imgs(k)
        axes[0].imshow(img, origin='lower')
        for i in range(4):
            axes[i+1].imshow(imgs[i], origin='lower')
    plot_imgs(axes, N - 1)
    def update_line(k):
        plot_imgs(axes, k)
        return []
    return animation.FuncAnimation(fig,
                                   update_line,
                                   frames=range(0, N, step),
                                   interval=0.01 * step * 1e3,
                                   blit=True)

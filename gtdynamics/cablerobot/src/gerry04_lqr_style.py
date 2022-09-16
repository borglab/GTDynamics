import gerry02_traj_tracking as gerry02
import draw_cdpr

import numpy as np
import gtdynamics as gtd
import matplotlib.pyplot as plt

def plot(ax, cdpr, controller, result, N, dt, des_T):
    """Plots the results"""
    # plot_trajectory(cdpr, result, dt*N, dt, N, des_T, step=1)

    act_T = [gtd.Pose(result, cdpr.ee_id(), k) for k in range(N+1)]
    act_xy = np.array([draw_cdpr.pose32xy(pose) for pose in act_T]).T
    des_xy = np.array([draw_cdpr.pose32xy(pose) for pose in des_T]).T

    # ls = draw_cdpr.draw_cdpr(ax, cdpr, gtd.Pose(result, cdpr.ee_id(), 0))
    # ls[1].remove()
    # for l in ls[2]:
    #     l.remove()
    # for l in ls[1:]:
    #     l.remove()
    ls = draw_cdpr.draw_traj(ax, cdpr, des_xy, act_xy)
    # plt.legend(ls, [r'$\bf{x}_d$', r'$\bf{x}_{ff}$'])
    # plt.legend(loc='upper right')


if __name__ == '__main__':
    # cProfile.run('results = main(N=100)', sort=SortKey.TIME)
    # fname = 'data/ATL_filled_output_7mps2.h'
    # fname = 'data/concentric_diamonds2_output_2mps_20mps2.h'
    fname = 'data/ATL_outline_output.h'
    Q, R = 1e-1, 1e-3
    results = gerry02.main(fname=fname, Q=np.ones(6)*Q, R=np.ones(1)*R, dN=1, debug=False, speed_multiplier=1)
    plot(*results)
    plt.show()

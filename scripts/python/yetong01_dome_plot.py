import numpy as np
import matplotlib.pyplot as plt

from yetong00_utils import load_data, draw_half_sphere

def main():
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    draw_half_sphere(ax)

    folder = "results/dome_estimation_simple/lm/"

    points = load_data(folder + "values_final.txt")


    ax.scatter(points[:, 0], points[:, 1], points[:, 2], color="blue")
    ax.plot3D(points[:, 0], points[:, 1], points[:, 2], color="blue")

    # ax.set_aspect("equal")
    ax.set_xlim3d([-1.8, 1.8])
    ax.set_ylim3d([-1.8, 1.8])
    ax.set_zlim3d([-1.8, 1.8])
    ax.set_box_aspect([1,1,1])
    ax.legend()
    plt.savefig(folder + "dome.pdf")
    plt.show()


if __name__ == "__main__":
    main()
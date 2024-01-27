import numpy as np
import matplotlib.pyplot as plt

from yetong00_utils import load_csv, draw_half_sphere

def main():
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    draw_half_sphere(ax)

    folder = "data/rss01_estimation/"
    points_result = load_csv(folder + "ielm_cr_values.csv")
    points_init = load_csv(folder + "init_values.csv")
    points_gt = load_csv(folder + "gt_values.csv")

    ax.scatter(points_result["x"], points_result["y"], points_result["z"], color="blue", s=5)
    ax.plot3D(points_result["x"], points_result["y"], points_result["z"], color="blue", label="result")

    ax.scatter(points_init["x"], points_init["y"], points_init["z"], color="red", s=5)
    ax.plot3D(points_init["x"], points_init["y"], points_init["z"], color="red", label="init est")

    ax.scatter(points_gt["x"], points_gt["y"], points_gt["z"], color="green", s=5)
    ax.plot3D(points_gt["x"], points_gt["y"], points_gt["z"], color="green", label="gt")

    # ax.set_aspect("equal")
    ax.set_xlim3d([-1.2, 1.2])
    ax.set_ylim3d([-1.2, 1.2])
    ax.set_zlim3d([-1.2, 1.2])
    ax.set_box_aspect([1,1,1])
    ax.legend()
    plt.savefig("figures/" + "dome.pdf")
    plt.show()


if __name__ == "__main__":
    main()
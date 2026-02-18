import numpy as np
import matplotlib.pyplot as plt

from yetong00_utils import load_data, draw_half_sphere


def main():
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    draw_half_sphere(ax)

    folder = "results/half_sphere_lm_opt/"

    initial_point = load_data(folder + "initial_values.txt")
    points = load_data(folder + "intermediate_values.txt")
    vectors = load_data(folder + "tangent_vectors.txt")

    ax.scatter([0], [-1], [-2], color="orange", label="attractor")
    ax.scatter(initial_point[:, 0], initial_point[:, 1], initial_point[:,2], color="black", label="start")
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], color="blue", label="steps")
    points = np.concatenate((initial_point, points), axis=0)
    for i in range(len(vectors)):
        start_pt = points[i]
        end_pt = points[i] + vectors[i]
        if i==0:
            ax.plot3D([start_pt[0], end_pt[0]], [start_pt[1], end_pt[1]], [start_pt[2], end_pt[2]], color="blue", label="delta")
        else:
            ax.plot3D([start_pt[0], end_pt[0]], [start_pt[1], end_pt[1]], [start_pt[2], end_pt[2]], color="blue")

    # ax.set_aspect("equal")
    ax.set_xlim3d([-1.8, 1.8])
    ax.set_ylim3d([-1.8, 1.8])
    ax.set_zlim3d([-1.8, 1.8])
    ax.set_box_aspect([1,1,1])
    ax.legend()
    plt.savefig(folder + "halfsphere_single.pdf")
    plt.show()


if __name__ == "__main__":
    main()
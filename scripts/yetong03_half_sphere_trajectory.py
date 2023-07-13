import numpy as np
import matplotlib.pyplot as plt
from yetong02_half_sphere_plot import load_data, draw_half_sphere

def main():
    folder = "results/half_sphere_traj_lm/"

    initial_point = load_data(folder + "initial_values.txt")
    points = load_data(folder + "intermediate_values.txt")
    vectors = load_data(folder + "tangent_vectors.txt")

    num_points = int(len(initial_point[0])/3)
    points = np.concatenate((initial_point, points), axis=0)
    points = points.reshape((points.shape[0], num_points, 3))
    print(points)

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    draw_half_sphere(ax)
    colors = ['r', 'orange', 'g', 'b', 'purple']
    for i in range(len(points)):
        ax.scatter(points[i, :, 0], points[i, :, 1], points[i, :, 2], color=colors[i])

    # ax.scatter(points[:, 0], points[:, 1], points[:, 2], color="black")
    # for i in range(len(vectors)):
    #     start_pt = points[i]
    #     end_pt = points[i] + vectors[i]
    #     ax.plot3D([start_pt[0], end_pt[0]], [start_pt[1], end_pt[1]], [start_pt[2], end_pt[2]], color="blue")

    ax.set_xlim3d([-1.5, 1.5])
    ax.set_ylim3d([-1.5, 1.5])
    ax.set_zlim3d([-1.5, 1.5])
    ax.set_box_aspect([1,1,1])
    ax.legend()
    plt.show()

    plt.show()


if __name__ == "__main__":
    main()

import numpy as np
import matplotlib.pyplot as plt

# load optimization result data from file
def load_data(filename):
    data = []
    with open(filename) as data_file:
        lines = data_file.readlines()
        for line in lines:
            line = line.split()
            line = [float(data) for data in line]
            data.append(line)

    return np.array(data)

def draw_half_sphere(ax):
    phi, theta = np.mgrid[0.0:0.5*np.pi:180j, 0.0:2.0*np.pi:720j] # phi = alti, theta = azi
    x = np.sin(phi)*np.cos(theta)
    y = np.sin(phi)*np.sin(theta)
    z = np.cos(phi)    
    #Set colours and render
    ax.plot_surface(
        x, y, z,  rstride=4, cstride=4, color='w', alpha=0.2, linewidth=0)  


def main():
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    draw_half_sphere(ax)

    folder = "results/half_sphere_lm_opt/"

    initial_point = load_data(folder + "initial_values.txt")
    points = load_data(folder + "intermediate_values.txt")
    vectors = load_data(folder + "tangent_vectors.txt")

    points = np.concatenate((initial_point, points), axis=0)
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], color="black")
    for i in range(len(vectors)):
        start_pt = points[i]
        end_pt = points[i] + vectors[i]
        ax.plot3D([start_pt[0], end_pt[0]], [start_pt[1], end_pt[1]], [start_pt[2], end_pt[2]], color="blue")

    # ax.set_aspect("equal")
    ax.set_xlim3d([-1.5, 1.5])
    ax.set_ylim3d([-1.5, 1.5])
    ax.set_zlim3d([-1.5, 1.5])
    ax.set_box_aspect([1,1,1])
    ax.legend()
    plt.show()


if __name__ == "__main__":
    main()
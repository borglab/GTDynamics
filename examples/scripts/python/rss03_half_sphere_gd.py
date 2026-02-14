import numpy as np
import matplotlib.pyplot as plt
from yetong00_utils import draw_half_sphere

pg = np.array([1, 0.2, 0.7])
x0 = np.array([0, -0.8, 0.6])
gravity = np.array([0, 0, -20])
G = 15

def set_axes_equal(ax):
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    """

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def evaluate_cost(x):
    r = pg - x
    r_norm = np.linalg.norm(r)
    potential_energy = -G/r_norm - gravity.dot(x)
    return potential_energy

def compute_gradient(x):
    r = pg - x
    r_norm = np.linalg.norm(r)
    magnitude = G/(r_norm ** 2)
    force = r * (magnitude / r_norm)
    force += gravity
    return -force

def compute_value():
    return


def proj2tspace(x, gradient):
    v_component = x * (np.dot(x, gradient))
    t_vec = gradient - v_component
    if abs(x[2]) <1e-5:
        if t_vec[2] < 0:
            t_vec[2] = 0 
    return t_vec

def retract(x, t_vec):
    x_new = x + t_vec
    x_new = x_new / np.linalg.norm(x_new)
    if x_new[2] < 0:
        x_new[2] = 0
        x_new = x_new / np.linalg.norm(x_new)
    return x_new

def line_search(x, gradient, descent_vec, t, alpha = 0.5, beta=0.5):
    for i in range(100):
        tangent_vec = t * descent_vec
        new_x = retract(x, tangent_vec)
        nonlinear_cost_change = evaluate_cost(x) - evaluate_cost(new_x)
        linear_cost_change = -gradient.dot(tangent_vec)
        model_fidelity = nonlinear_cost_change/ linear_cost_change
        print("=========================================")
        print("tangent_vec", tangent_vec)
        print("new_x", new_x)
        print("nonlinear_cost_change", nonlinear_cost_change)
        print("linear_cost_change", linear_cost_change)
        print("model_fidelity", model_fidelity)
        if (model_fidelity > alpha):
            t *= 4
            return t, tangent_vec, new_x
        t *= beta

def optimize():
    t = 0.1
    x = x0
    x_list = [x]
    tangent_vec_list = []

    for i in range(4):
        gradient = compute_gradient(x)
        descent_vec = proj2tspace(x, -gradient)
        t, tangent_vec, new_x = line_search(x, gradient, descent_vec, t)
        x = new_x
        tangent_vec_list.append(tangent_vec)
        x_list.append(x)
    return x_list, tangent_vec_list

x_list, tangent_vec_list = optimize()
x_list = np.array(x_list)
tangent_vec_list = np.array(tangent_vec_list)
print(x_list)
print(tangent_vec_list)

# x_list = np.array(x_list)

fig = plt.figure(figsize=(8, 6))
ax = plt.axes(projection='3d')
draw_half_sphere(ax)

ax.scatter([x0[0]], [x0[1]], [x0[2]], color="black", label="start")
ax.scatter([pg[0]], [pg[1]], [pg[2]], color="orange", label="attractor")
ax.scatter(x_list[1:,0], x_list[1:,1], x_list[1:,2], label="optimization progress")
for i in range(len(tangent_vec_list)):
    tangent_vec = tangent_vec_list[i]
    pt = x_list[i]
    pt_tv = pt + tangent_vec
    next_pt = x_list[i+1]
    print("pt", pt)
    print("next_pt", next_pt)
    if i==0:
        ax.plot3D([pt[0], pt_tv[0]], [pt[1], pt_tv[1]], [pt[2], pt_tv[2]], color="blue", label="tangent_vector")
        ax.plot3D([pt_tv[0], next_pt[0]], [pt_tv[1], next_pt[1]], [pt_tv[2], next_pt[2]], color="green", label="retraction")
    else:
        ax.plot3D([pt[0], pt_tv[0]], [pt[1], pt_tv[1]], [pt[2], pt_tv[2]], color="blue")
        ax.plot3D([pt_tv[0], next_pt[0]], [pt_tv[1], next_pt[1]], [pt_tv[2], next_pt[2]], color="green")

# ax.plot3D(x_list[:,0], x_list[:,1], x_list[:,2], label="optimization progress")


ax.axes.set_aspect("auto")
ax.set_box_aspect([1,1,1])
ax.axes.set_xlim3d(left=-1.1, right=1.1) 
ax.axes.set_ylim3d(bottom=-1.1, top=1.1) 
ax.axes.set_zlim3d(bottom=-0.1, top=1.1) 

ax.grid(False)
ax.set_xticks([])
ax.set_yticks([])
ax.set_zticks([])
# make the panes transparent
# ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
# ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
# ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
# make the grid lines transparent
# ax.xaxis._axinfo["grid"]['color'] =  (1,1,1,0)
# ax.yaxis._axinfo["grid"]['color'] =  (1,1,1,0)
# ax.zaxis._axinfo["grid"]['color'] =  (1,1,1,0)
ax.set_axis_off()
plt.tight_layout()

ax.view_init(15, -55)

set_axes_equal(ax)
ax.legend()
# ax.legend(loc=[0.9,0.4])
plt.savefig('figures/toy_half_sphere.pdf')
plt.show()

import numpy as np
import matplotlib.pyplot as plt
import csv

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


def load_csv(file_name):
    data = []
    with open(file_name, newline='') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
        rows = [row for row in spamreader]
        title = rows[0]
        for row in rows[1:]:
            data_row = [float(d) for d in row]
            data.append(data_row)
    data = np.array(data)
    data = {title[i]: data[:, i] for i in range(len(title))}
    return data


def load_csv_no_title(file_name):
    with open(file_name, newline='') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
        data = [row for row in spamreader]
        return data


def draw_half_sphere(ax):
    phi, theta = np.mgrid[0.0:0.5*np.pi:180j,
                          0.0:2.0*np.pi:720j]  # phi = alti, theta = azi
    x = np.sin(phi)*np.cos(theta)
    y = np.sin(phi)*np.sin(theta)
    z = np.cos(phi)
    # Set colours and render
    ax.plot_surface(
        x, y, z,  rstride=4, cstride=4, color='w', alpha=0.2, linewidth=0)


def plot_error_vs_constraint(ax, scenario_folder: str):
    barrier_data = load_csv(scenario_folder + "barrier_summary_outerloop.csv")
    manopt_data = load_csv(scenario_folder + "manopt_states.csv")

    cost_barrier = barrier_data["cost"]
    constraint_vio_barrier = barrier_data["e_violation"] ** 2 + \
        barrier_data["i_violation"] ** 2

    cost_manopt = manopt_data["error"][-1]
    constraint_vio_manopt = 1e-10

    ax.plot(cost_barrier, constraint_vio_barrier, color="b", label="barrier")
    ax.scatter(cost_barrier, constraint_vio_barrier, color="b")
    ax.scatter([cost_manopt], [constraint_vio_manopt],
               color="r", label="manopt")

    ax.set_xlabel("cost")
    ax.set_ylabel("constraint violation")
    ax.set_yscale('log')
    ax.set_title("cost vs. constraint violation")
    ax.legend()


def plot_optimization_progress(scenario_folder: str, exp_names, ax_cost=None, ax_constraint=None, ax_proj_cost=None):
    for exp_name in exp_names:
        file_path = scenario_folder + exp_name + "_progress.csv"
        data = load_csv(file_path)
        if ax_cost is not None:
            ax_cost.plot(data["iterations"], data["cost"], label=exp_name)
        if ax_constraint is not None:
            ax_constraint.plot(data["iterations"],
                               data["e_violation"] ** 2 + data["i_violation"] ** 2)
        if ax_proj_cost is not None:
            ax_proj_cost.plot(data["iterations"],
                              data["proj_cost"], label=exp_name)


def plot_trajectory(axs, scenario_folder):
    init_data = load_csv(scenario_folder + "init_traj.csv")
    manopt_data = load_csv(scenario_folder + "manopt_traj.csv")
    # barrier_data = load_csv(scenario_folder + "barrier_traj.csv")

    data_list = [init_data, manopt_data]
    line_styles = ['solid', 'dashed']
    joints = ["fl_upper", "fl_lower", "rl_upper",
              "rl_lower", "fl_hip", "rl_hip"]
    colors = ["r", 'g', "b", "c", "orange", "purple"]

    for data, line_style in zip(data_list, line_styles):
        for joint, color in zip(joints, colors):
            axs[0, 0].plot(data["time"], data[joint+"_q"],
                           color=color, linestyle=line_style)
            axs[0, 1].plot(data["time"], data[joint+"_v"],
                           color=color, linestyle=line_style)
            axs[1, 0].plot(data["time"], data[joint+"_a"],
                           color=color, linestyle=line_style)
            axs[1, 1].plot(data["time"], data[joint+"_T"],
                           color=color, linestyle=line_style)

        axs[2, 0].plot(data["time"], data["base_y"], linestyle=line_style)
        axs[2, 1].plot(data["time"], data["base_z"], linestyle=line_style)

    axs[0, 0].set_title("joint angle")
    axs[0, 1].set_title("joint velocity")
    axs[1, 0].set_title("joint acceleration")
    axs[1, 1].set_title("joint torque")
    axs[2, 0].set_title("torso pose x")
    axs[2, 1].set_title("torso pose y")

import numpy as np
import matplotlib.pyplot as plt
import matplotlib

def draw_cost_countours(axes, center, sigmas):
    for i in range(5):
        ellipse = matplotlib.patches.Ellipse(xy = center, width = 0.1*sigmas[0]*i, height=0.1*sigmas[1]*i, fill=False)
        axes.add_artist(ellipse)


class State:
    point = []
    lambda_value = 0
    error = 0
    num_iters = 0


class Trial:
    tv = []
    new_point = []
    lambda_value = 0
    new_error = 0
    step_is_successful = 0

def load_summary(folder_path):
    file_name = folder_path + "summary.txt"
    with open(file_name) as data_file:
        lines = data_file.readlines()
        init_point = [float(data) for data in lines[0].split()]
        goal_point = [float(data) for data in lines[1].split()]
        sigmas = [float(data) for data in lines[2].split()]
        num_iters = int(lines[3])
    return init_point, goal_point, sigmas, num_iters

def load_iter(folder_path, iter_id):
    file_name = folder_path + "state_{}.txt".format(iter_id)
    state = State()
    with open(file_name) as data_file:
        lines = data_file.readlines()
        state.point = [float(data) for data in lines[0].split()]
        state.lambda_value = float(lines[1])
        state.error = float(lines[2])
        state.num_trials = int(lines[3])

    trials = []
    for trial_id in range(state.num_trials):
        file_name = folder_path + "trial_{}_{}.txt".format(iter_id, trial_id)
        trial = Trial()
        with open(file_name) as data_file:
            lines = data_file.readlines()
            trial.tv = [float(data) for data in lines[0].split()]
            trial.new_point = [float(data) for data in lines[1].split()]
            trial.lambda_value = float(lines[2])
            trial.new_error = float(lines[3])
            trial.step_is_successful = int(lines[4])
        trials.append(trial)

    return state, trials


def make_plot(folder_path, axes, title):
    
    init_point, goal_point, sigmas, num_iters = load_summary(folder_path)

    axes.scatter([init_point[0]], [init_point[1]], color='k')
    axes.scatter([goal_point[0]], [goal_point[1]], color='orange')

    label_done = False
    for iter_id in range(num_iters):
        state, trials = load_iter(folder_path, iter_id)
        # axes.scatter([state.point[0]], [state.point[1]], color='b')
        
        for trial in trials:
            color = 'g'
            if label_done == False:
                axes.plot([state.point[0], state.point[0] + trial.tv[0]], [state.point[1], state.point[1] + trial.tv[1]], color='b', label="tangent vector")
                axes.plot([state.point[0] + trial.tv[0], trial.new_point[0]], [state.point[1] + trial.tv[1], trial.new_point[1]], color=color, label="retraction")
                label_done = True
            else:
                axes.plot([state.point[0], state.point[0] + trial.tv[0]], [state.point[1], state.point[1] + trial.tv[1]], color='b')
                axes.plot([state.point[0] + trial.tv[0], trial.new_point[0]], [state.point[1] + trial.tv[1], trial.new_point[1]], color=color)
            if trial.step_is_successful == 1:
                axes.scatter([trial.new_point[0]], [trial.new_point[1]], color='b')

    axes.set_xlim(-4, 2)
    axes.set_ylim(-2, 3)
    axes.set_aspect(1)
    manifold_circle = plt.Circle((0, 0), 1, color='r', fill=False)
    axes.add_artist(manifold_circle)

    axes.set_title(title)
    axes.legend()

    draw_cost_countours(axes, goal_point, sigmas)


figure, axes = plt.subplots(1, 2, figsize=(12,5))

folder_path = "data/special_proj_so2/"
make_plot(folder_path, axes[0], "cost-aware projection")

folder_path = "data/normal_proj_so2/"
make_plot(folder_path, axes[1], "standard projection")

plt.savefig("special_proj.pdf")
plt.show()

import numpy as np
import matplotlib.pyplot as plt
from yetong00_utils import load_csv

folder = "data/yetong08_e_quadruped_jump/"

exp_names = []
exp_names.append("Orthonormal_Barrier_all")
exp_names.append("Orthonormal_Barrier_cost")
# exp_names.append("Orthonormal_Barrier_costscale")
# exp_names.append("Orthonormal_Barrier_basisT")

legends = ["standard projection", "cost-aware projection"]

colors = ["blue", "red"]

# exp_names.append("Orthonormal_Hierarchical_all")
# exp_names.append("Orthonormal_Hierarchical_cost")
# exp_names.append("Orthonormal_Hierarchical_costscale")
# exp_names.append("Orthonormal_Hierarchical_basisT")

# exp_names.append("EliminationT_Barrier_all")
# exp_names.append("EliminationT_Barrier_cost")
# exp_names.append("EliminationT_Barrier_costscale")
# exp_names.append("EliminationT_Barrier_basisT")

# exp_names.append("EliminationT_Hierarchical_all")
# exp_names.append("EliminationT_Hierarchical_cost")
# exp_names.append("EliminationT_Hierarchical_costscale")
# exp_names.append("EliminationT_Hierarchical_basisT")

exp_names_2nd = []
exp_names_2nd.append("Orthonormal_Barrier_all_Orthonormal_Barrier_all")
exp_names_2nd.append("Orthonormal_Barrier_all_Orthonormal_Barrier_cost")

figure, axes = plt.subplots(1, 2, figsize=(12, 5))

for exp_name, label, color in zip(exp_names, legends, colors):
    state_file_path = folder + exp_name + "_states.csv"
    trial_file_path = folder + exp_name + "_trials.csv"

    states = load_csv(state_file_path)
    trials = load_csv(trial_file_path)

    # iterations = []
    # step_sizes = []
    
    # for trial in trials:
    #     if trial["step_is_successful"]:
    #         iterations.append(trial["iterations"])
    #         step_sizes.append(trial["tangent_vector_norm"])

    axes[0].plot(states["iterations"], states["error"], label=label,color=color)
    indices = np.where(trials["step_is_successful"])[0]
    axes[1].plot(trials["iterations"][indices], trials["tangent_vector_norm"][indices], label=label, color=color)


for exp_name, label, color in zip(exp_names_2nd, legends, colors):
    state_file_path = folder + exp_name + "_states.csv"
    trial_file_path = folder + exp_name + "_trials.csv"

    states = load_csv(state_file_path)
    trials = load_csv(trial_file_path)

    axes[0].plot(states["iterations"]+100, states["error"],'--', color=color, label=label)
    indices = np.where(trials["step_is_successful"])[0]
    axes[1].plot(trials["iterations"][indices]+100, trials["tangent_vector_norm"][indices], '--', color=color, label=label)

axes[0].set_xlabel('iterations')
axes[0].set_yscale('log')
axes[0].set_ylabel('error')
axes[0].set_title('error vs. iterations')
axes[1].set_xlabel('iterations')
axes[1].set_yscale('log')
axes[1].set_ylabel('step size')
axes[1].set_title('step size over iterations')
axes[1].legend()

plt.savefig(folder + "retraction_comparision.png")

plt.show()

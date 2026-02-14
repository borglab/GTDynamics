import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.animation as animation
from yetong00_utils import load_csv, load_csv_no_title

folder = "data/yetong10_ie_quadruped_jump/progress/"
states_cost_data = load_csv(folder+"states_cost_eval.csv")
# trials_cost_data = load_csv(folder+"trials_cost_eval.csv")
constraint_data = load_csv_no_title(folder+"active_constraints.csv")
states_info = load_csv(folder+"states_info.csv")
trials_info = load_csv(folder+"trials_info.csv")

# cost -log scale
# cost stack plot

# retraction fidelity (linear update by retraction vector / linear update by tangent vector)
# cost function fidelity (nonlinear cost change by retraction vector / linear cost chagne by retraction vector)
# overall model fidelity (nonlinear cost change by retraction vector / linear update by tangent vector)
# overall model fidelity = retraction fidelity * cost function fidelity 

# active constraints over iterations shown as a grid


fig, ax = plt.subplots(2, 2, figsize=(12, 9))
ax_cost = ax[0, 0]
ax_cost_stack = ax[1, 0]
ax_tv = ax[0, 1]
ax_fidelity = ax[1, 1]


cost_names = []
cost_terms = []
iters = states_cost_data['iterations']
cost_total = []
for cost_name, cost_term in states_cost_data.items():
  if cost_name != 'iterations':
    if (len(cost_total)==0):
      cost_total = cost_term
    else:
      cost_total = cost_total + cost_term
    cost_names.append(cost_name)
    cost_terms.append(cost_term)

# plot cost
for cost_name, cost_term in zip(cost_names, cost_terms):
  ax_cost.plot(iters, cost_term, label=cost_name)
ax_cost.plot(iters, cost_total, label="total")
ax_cost.set_ylim([1e-1, 1e8])
ax_cost.legend()
ax_cost.set_yscale('log')
ax_cost.set_title('cost')

# plot cost stack
start_iter = 15
cliped_cost_terms = []
for cost_term in cost_terms:
  cliped_cost_terms.append(cost_term[start_iter:])
ax_cost_stack.stackplot(iters[start_iter:], cliped_cost_terms, labels=cost_names)
ax_cost_stack.legend()
ax_cost_stack.set_title('cost')

# plot tangent vector size
indices = np.where(trials_info["step_is_successful"])[0]
ax_tv.plot(trials_info["iterations"][indices], trials_info["tangent_vector_norm"][indices])
ax_tv.set_yscale('log')
ax_tv.set_title('linear update norm')

# plot fidelity
# retraction_fidelity = trials_info["linear_cost_change_with_retract_delta"] / trials_info["linear_cost_change"]
# cost_fidelity = trials_info["nonlinear_cost_change"] / trials_info["linear_cost_change_with_retract_delta"]
# overall_fidelity = trials_info["nonlinear_cost_change"] / trials_info["linear_cost_change"]

# ax_fidelity.plot(trials_info["iterations"][indices], retraction_fidelity[indices], label="retraction")
# ax_fidelity.plot(trials_info["iterations"][indices], cost_fidelity[indices], label="cost")
# ax_fidelity.plot(trials_info["iterations"][indices], overall_fidelity[indices], label="overall")
# ax_fidelity.set_title('model fidelity')

plt.show()



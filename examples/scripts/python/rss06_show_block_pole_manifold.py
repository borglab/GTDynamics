import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import numpy as np
from yetong00_utils import load_csv

class KinoDynamicPendulum:
  def __init__(self):
    self.m = 1
    self.M = 1
    self.r = 1
    self.g = 10
    self.mu = 0.8
    self.tau_min = -25
    self.tau_max = 25

  def torqueConstraint1(self, theta, w):
    return self.tau_min / (self.m * self.r ** 2) - self.g/self.r * np.cos(theta)
  
  def torqueConstraint2(self, theta, w):
    return self.tau_max / (self.m * self.r ** 2) - self.g/self.r * np.cos(theta)
  
  def frictionConeConstraint1(self, theta, w):
    pos_idx = self.mu * np.cos(theta) + np.sin(theta) > 0
    neg_idx = self.mu * np.cos(theta) + np.sin(theta) < 0
    value = (-self.mu * (1+self.M/self.m) * self.g/self.r + self.mu * w**2 * np.sin(theta) - w**2 * np.cos(theta)) / (self.mu * np.cos(theta) + np.sin(theta))
    return value, pos_idx, neg_idx
  
  def frictionConeConstraint2(self, theta, w):
    pos_idx = self.mu * np.cos(theta) - np.sin(theta) > 0
    neg_idx = self.mu * np.cos(theta) - np.sin(theta) < 0
    value = (-self.mu * (1+self.M/self.m) * self.g/self.r + self.mu * w**2 * np.sin(theta) + w**2 * np.cos(theta)) / (self.mu * np.cos(theta) - np.sin(theta))
    return value, pos_idx, neg_idx
  

fig, ax = plt.subplots(subplot_kw={"projection": "3d"})


# Make data.
theta_list = np.arange(0, 2*np.pi, 0.001 * np.pi)
w_list = np.arange(-20, 20, 0.01)
theta, w = np.meshgrid(theta_list, w_list)

# constraints
pen = KinoDynamicPendulum()
lower_bound = pen.torqueConstraint1(theta, w)
upper_bound = pen.torqueConstraint2(theta, w)
fc1, fc1_pos_idx, fc1_neg_idx = pen.frictionConeConstraint1(theta, w)
fc2, fc2_pos_idx, fc2_neg_idx = pen.frictionConeConstraint2(theta, w)

fc1_lb_valid = np.logical_and(fc1_pos_idx, fc1 > lower_bound)
lower_bound[fc1_lb_valid] = fc1[fc1_lb_valid]
fc2_lb_valid = np.logical_and(fc2_pos_idx, fc2 > lower_bound)
lower_bound[fc2_lb_valid] = fc2[fc2_lb_valid]
fc1_lb_valid = np.logical_and(fc1_lb_valid, np.logical_not(fc2_lb_valid))
tl_lb_valid = np.logical_not(np.logical_or(fc1_lb_valid, fc2_lb_valid))

fc1_ub_valid = np.logical_and(fc1_neg_idx, fc1 < upper_bound)
upper_bound[fc1_ub_valid] = fc1[fc1_ub_valid]
fc2_ub_valid = np.logical_and(fc2_neg_idx, fc2 < upper_bound)
upper_bound[fc2_ub_valid] = fc2[fc2_ub_valid]
fc1_ub_valid = np.logical_and(fc1_ub_valid, np.logical_not(fc2_ub_valid))
tl_ub_valid = np.logical_not(np.logical_or(fc1_ub_valid, fc2_ub_valid))

color_tl1 = np.array([0, 1, 0, 0.4])
color_tl2 = np.array([0, 1, 0, 0.4])
color_fc1 = np.array([1, 0, 0, 0.4])
color_fc2 = np.array([0, 0, 1, 0.4])


invalid = lower_bound > upper_bound
lower_bound[invalid] = np.nan
upper_bound[invalid] = np.nan

fc1_valid = np.logical_or(fc1_lb_valid, fc1_ub_valid)
# fc1_valid=fc1_lb_valid
fc2_valid = np.logical_or(fc2_lb_valid, fc2_ub_valid)

fc1_invalid = np.logical_or(invalid, np.logical_not(fc1_valid))
fc2_invalid = np.logical_or(invalid, np.logical_not(fc2_valid))
tl_lb_invalid = np.logical_or(invalid, np.logical_not(tl_lb_valid))
tl_ub_invalid = np.logical_or(invalid, np.logical_not(tl_ub_valid))

fc1[fc1_invalid] = np.nan
fc2[fc2_invalid] = np.nan
lower_bound[tl_lb_invalid] = np.nan
upper_bound[tl_ub_invalid] = np.nan

surf1 = ax.plot_surface(theta, w, fc1, color = color_fc1,
                       linewidth=0)

surf2 = ax.plot_surface(theta, w, lower_bound, color = color_tl1,
                       linewidth=0)

surf3 = ax.plot_surface(theta, w, fc2, color = color_fc2,
                       linewidth=0)

surf4 = ax.plot_surface(theta, w, upper_bound, color = color_tl1,
                       linewidth=0)

# data = load_data("data/gd.txt")

# ax.scatter([0], [0], [0], c='k')
# ax.scatter([4.4], [10], [10], c='k')



# scale = 1e0
# scale1 = 1.0
# i = 80
# # ax.scatter(data[i, 0], data[i, 1], data[i, 2], c='k')
# ax.scatter(data[:i, 0], data[:i, 1], data[:i, 2], c='k')
# ax.plot([data[i, 0], data[i,0]+data[i+1,6]*scale], [data[i, 1], data[i,1]+data[i+1,7]*scale], [data[i, 2], data[i,2]+data[i+1,8]*scale])
# ax.plot([data[i, 0], data[i,0]+data[i+1,3]*scale1], [data[i, 1], data[i,1]+data[i+1,4]*scale1], [data[i, 2], data[i,2]+data[i+1,5]*scale1])
# ax.scatter([1.1], [2.7], [17], c='k')
# ax.scatter([1.3], [1.9], [17], c='k')

# ax.scatter([0.806292], [3.42574], [6.55336])

# # Customize the z axis.
# ax.set_zlim(-100, 100)
# ax.zaxis.set_major_locator(LinearLocator(10))
# # A StrMethodFormatter is used automatically
# ax.zaxis.set_major_formatter('{x:.02f}')

# Add a color bar which maps values to colors.
# fig.colorbar(surf, shrink=0.5, aspect=5)

# fig2 = plt.figure()
# ax2 = fig2.add_subplot(111)
# ax2.imshow(invalid, aspect='auto', cmap=plt.cm.gray, interpolation='nearest')
# plt.grid(False)
ax.set_xlabel("q")
ax.set_ylabel("v")
ax.set_zlabel("a")


filename = "data/rss03_cp_friction/ielm_traj.csv"
data = load_csv(filename)
ax.plot(data["q"], data["v"], data["a"], color="k", linewidth=2, label="CMC-Opt")
ax.view_init(35, 80)
ax.legend(loc = [0.7,0.8])

plt.tight_layout()
plt.savefig("figures/cp_manifold.pdf")
plt.show()

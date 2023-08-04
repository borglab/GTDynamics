import numpy as np
import matplotlib.pyplot as plt
from yetong00_utils import load_data

filename = "results/pole_lm/values_final.txt"
values = load_data(filename)


fig, axs= plt.subplots(1, 3, figsize=(12,4))
# show pole trajectory
axs[0].set_title("pole trajectory")
axs[0].scatter(np.cos(values[:,0]), np.sin(values[:,0]))

# show plot of q, v, a
axs[1].plot(values[:,0], label="q")
axs[1].plot(values[:,1], label="v")
axs[1].plot(values[:,2], label="a")
axs[1].legend()

# show contact force in friction cone
mu=0.8
scale = 30
axs[2].set_title("friction cone")
axs[2].plot(values[:,4], values[:,5])
axs[2].plot([0, scale * mu], [0, scale], color='b', linewidth=0.5)
axs[2].plot([0, -scale * mu], [0, scale], color='b', linewidth=0.5)


plt.show()

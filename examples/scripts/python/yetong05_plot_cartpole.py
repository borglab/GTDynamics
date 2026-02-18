import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.animation as animation
from yetong00_utils import load_csv

data = load_csv("data/ineq_cartpole_traj.csv")

fig = plt.figure()

colormap = mpl.colormaps["plasma"]

frames = []
plots = []

for i in range(len(data)):
  x = data[i, 1]
  theta = data[i, 5]
  print(x, theta)
  p1 = (x, 0)
  p2 = (x+np.sin(theta), -np.cos(theta))
  frame = plt.plot((p1[0], p2[0]), (p1[1], p2[1]), color = colormap(i/len(data)), animated=True)
  frames.append(frame[0])
  plots.append(list(frames))

ani = animation.ArtistAnimation(fig, plots, interval=50, blit=True,
                                repeat_delay=1000)
# plt.savefig("figures/cart_pole.pdf")
plt.show()


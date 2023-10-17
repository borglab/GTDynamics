import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import csv
import matplotlib.animation as animation

def load_csv(file_name):
  data = []
  with open(file_name, newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
    rows = [row for row in spamreader]
    for row in rows[1:]:
        data_row = [float(d) for d in row]
        data.append(data_row)
  data = np.array(data)
  return data

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

plt.show()


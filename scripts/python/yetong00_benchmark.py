import numpy as np
import matplotlib.pyplot as plt
from yetong00_utils import load_data

data_file = "data/ineq_cartpole_barrier.txt"
data = load_data(data_file)

cost_barrier = data[:,2]
constraint_vio_barrier = data[:,3] ** 2 + data[:,4] ** 2

cost_manopt = 92.249
constraint_vio_manopt = 0.0

plt.plot(cost_barrier, constraint_vio_barrier)
plt.scatter([cost_manopt], [constraint_vio_manopt])
# plt.yscale("log")
plt.show()

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# load date
def load_data(file_path):
    df = pd.read_csv(file_path)
    jangles_list = []
    jvels_list = []
    jaccels_list = []
    jtorques_list = []

    for i in range(len(df)):
        jangles = [df.loc[i][str(j)] for j in range(12)]
        jvels = [df.loc[i][str(j) + '.1'] for j in range(12)]
        jaccels = [df.loc[i][str(j) + '.2'] for j in range(12)]
        jtorques = [df.loc[i][str(j) + '.3'] for j in range(12)]

        # print(jangles)
        jangles_list.append(jangles)
        jvels_list.append(jvels)
        jaccels_list.append(jaccels)
        jtorques_list.append(jtorques)

    jangles_array = np.array(jangles_list)
    jvels_array = np.array(jvels_list)
    jaccels_array = np.array(jaccels_list)
    jtorques_array = np.array(jtorques_list)

    return jangles_array, jvels_array, jaccels_array, jtorques_array

def make_plot(axs, jangles_array, jvels_array, jaccels_array, jtorques_array, idx, label):
    axs[0, 0].plot(jangles_array[:, idx], label = label)
    axs[0, 1].plot(jvels_array[:, idx], label = label)
    axs[1, 0].plot(jaccels_array[:, idx], label = label)
    axs[1, 1].plot(jtorques_array[:, idx], label = label)
    axs[0, 0].set_title("q")
    axs[0, 1].set_title("v")
    axs[1, 0].set_title("a")
    axs[1, 1].set_title("T")

fig, axs = plt.subplots(2, 2)
idx = 0
file_path = '/Users/yetongzhang/packages/GTDynamics/data/init_traj.csv'
jangles_array, jvels_array, jaccels_array, jtorques_array = load_data(file_path)
make_plot(axs, jangles_array, jvels_array, jaccels_array, jtorques_array, idx, "init")
file_path = '/Users/yetongzhang/packages/GTDynamics/data/cm_traj.csv'
jangles_array, jvels_array, jaccels_array, jtorques_array = load_data(file_path)
make_plot(axs, jangles_array, jvels_array, jaccels_array, jtorques_array, idx, "manifold")
file_path = '/Users/yetongzhang/packages/GTDynamics/data/soft_traj.csv'
jangles_array, jvels_array, jaccels_array, jtorques_array = load_data(file_path)
make_plot(axs, jangles_array, jvels_array, jaccels_array, jtorques_array, idx, "soft")
axs[0, 1].legend()
fig.suptitle("hip joint")


fig, axs = plt.subplots(2, 2)
idx = 1
file_path = '/Users/yetongzhang/packages/GTDynamics/data/init_traj.csv'
jangles_array, jvels_array, jaccels_array, jtorques_array = load_data(file_path)
make_plot(axs, jangles_array, jvels_array, jaccels_array, jtorques_array, idx, "init")
file_path = '/Users/yetongzhang/packages/GTDynamics/data/cm_traj.csv'
jangles_array, jvels_array, jaccels_array, jtorques_array = load_data(file_path)
make_plot(axs, jangles_array, jvels_array, jaccels_array, jtorques_array, idx, "manifold")
file_path = '/Users/yetongzhang/packages/GTDynamics/data/soft_traj.csv'
jangles_array, jvels_array, jaccels_array, jtorques_array = load_data(file_path)
make_plot(axs, jangles_array, jvels_array, jaccels_array, jtorques_array, idx, "soft")
axs[0, 1].legend()
fig.suptitle("knee joint")

plt.show()


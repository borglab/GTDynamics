import numpy as np
import matplotlib.pyplot as plt

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

def draw_half_sphere(ax):
    phi, theta = np.mgrid[0.0:0.5*np.pi:180j, 0.0:2.0*np.pi:720j] # phi = alti, theta = azi
    x = np.sin(phi)*np.cos(theta)
    y = np.sin(phi)*np.sin(theta)
    z = np.cos(phi)    
    #Set colours and render
    ax.plot_surface(
        x, y, z,  rstride=4, cstride=4, color='w', alpha=0.2, linewidth=0)  


# def plot_loss(data):
#     fig = plt.figure()
#     plt.plot(data[:,0], data[:,1])

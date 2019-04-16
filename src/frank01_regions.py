"""Explore joint-space regions idea."""

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # pylint: disable=W0611

N = 60  # number of samples per joint angle


def torus(theta2, theta3):
    """Lift to torus space."""
    r, R = 0.7, 1.8
    x = (R + r * np.cos(theta3)) * np.cos(theta2)
    y = (R + r * np.cos(theta3)) * np.sin(theta2)
    z = r * np.sin(theta3)
    return x, y, z


def forward(theta1, theta2, theta3):
    """Forward kinematics."""
    # planar piece:
    theta23 = theta2 + theta3
    r = 3*np.cos(theta2) + 5*np.cos(theta23)
    z = 3*np.sin(theta2) + 5*np.sin(theta23)
    # rotate
    x = r*np.cos(theta1)
    y = r*np.sin(theta1)
    return x, y, z


class RegionsRRR():
    """ Create an RRR arm and create three regions-related figures:
        1. 2D j2-j3 space
        2. Corresponding torus
        3. Regions in 3D workspace
    """

    def __init__(self, joint_limits=True):
        """Pre-calculate a number of things."""
        # create joint angle sampling
        self.joint_limits = joint_limits
        if joint_limits:
            t2 = np.linspace(-np.radians(70), np.radians(87), N)
            t3 = np.linspace(-np.radians(129), np.radians(129), N)
        else:
            t2 = np.linspace(-np.pi, np.pi, 60)
            t3 = t2
        self.theta2, self.theta3 = np.meshgrid(t2, t3)
        acos = np.arccos(-np.cos(self.theta2)*3/5)
        t23 = self.theta2 + self.theta3

        # Assign region colors
        self.colors = np.empty(self.theta2.shape, dtype=str)
        for j, th2 in enumerate(t2):
            for i, th3 in enumerate(t3):
                if th3 >= 0:
                    c = 'green' if t23[i, j] > acos[i, j] else 'red'
                else:
                    c = 'cyan' if t23[i, j] < -acos[i, j] else 'blue'
                self.colors[i, j] = c

    def create_2d_figure(self):
        """ Create a figure showing regions in 2D joint space, 
            corresponding to joints 2 and three. We don't care about joint 1
            for the sake of singularities.
        """
        fig, axes = plt.subplots()
        axes.scatter(self.theta2, self.theta3,
                     alpha=0.1, c=self.colors.flatten())

        axes.set_xlabel(r'$\theta_2$', fontsize=15)
        axes.set_ylabel(r'$\theta_3$', fontsize=15)
        axes.set_title('Joint angles 2 and 3')

        axes.grid(True)
        axes.set_aspect('equal', 'box')
        fig.tight_layout()

    def create_torus_figure(self):
        """ Create a figure showing the same regions on a torus.
        """

        X, Y, Z = torus(self.theta2, self.theta3)

        # figure
        fig = plt.figure()
        axes = fig.gca(projection='3d')
        axes.set_axis_off()
        axes.set_xlim3d(-1.5, 1.5)
        axes.set_ylim3d(-1.5, 1.5)
        axes.set_zlim3d(-1, 1)
        axes.plot_surface(X, Y, Z, facecolors=self.colors,
                          alpha=0.1, rstride=1, cstride=1)
        axes.set_title('Joint angles 2 and 3')

        # Zero
        theta3 = self.theta3[:, 0]
        theta2 = 0 * theta3
        x, y, z = torus(theta2, theta3)
        axes.plot(x, y, z, 'green')

        # Elbow singularity
        theta2 = self.theta2[0, :]
        theta3 = 0 * theta2
        x, y, z = torus(theta2, theta3)
        axes.plot(x, y, z, 'cyan')

        # Shoulder singularity, positive arccos
        acos = np.arccos(-np.cos(theta2)*3/5)
        theta3 = acos-theta2
        if self.joint_limits:
            good = theta3 <= np.radians(129)
            x, y, z = torus(theta2[good], theta3[good])
        else:
            x, y, z = torus(theta2, theta3)
        axes.plot(x, y, z, 'red')

        # Shoulder singularity, negative arccos
        theta3 = -acos-theta2
        if self.joint_limits:
            good = theta3 >= -np.radians(129)
            x, y, z = torus(theta2[good], theta3[good])
        else:
            x, y, z = torus(theta2, theta3)
        axes.plot(x, y, z, 'red')

    def create_ws_figure(self):
        """ Create a figure showing the colored regions in the robot's workspace.
        """
        fig, axes = plt.subplots()
        theta1 = 0
        X, Y, Z = forward(theta1, self.theta2, self.theta3)
        axes.scatter(X, Z,
                     alpha=0.1, c=self.colors.flatten())
        axes.set_xlabel(r'$x$', fontsize=15)
        axes.set_ylabel(r'$z$', fontsize=15)
        axes.set_title(r'Workspace, $\theta_1=0$')
        axes.set_aspect('equal', 'box')
        fig.tight_layout()

    def create_3d_figure(self):
        """ Create a figure showing the colored regions in the robot's workspace.
        """
        fig = plt.figure()
        axes = fig.gca(projection='3d')
        # axes.set_axis_off()
        for theta1 in np.linspace(0, np.radians(92), 2):
            X, Y, Z = forward(theta1, self.theta2, self.theta3)
            axes.scatter(X, Y, Z, self.theta3,
                         alpha=0.1, c=self.colors.flatten())

        axes.set_xlabel(r'$x$', fontsize=15)
        axes.set_ylabel(r'$y$', fontsize=15)
        axes.set_title('Workspace')
        axes.set_aspect('equal', 'box')
        fig.tight_layout()

    def run(self):
        """Execute and create three figures."""
        self.create_2d_figure()
        self.create_torus_figure()
        self.create_ws_figure()
        self.create_3d_figure()
        plt.show()


if __name__ == "__main__":
    RegionsRRR().run()

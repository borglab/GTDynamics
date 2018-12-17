"""
Utilities for Fetch examples
Author: Frank Dellaert
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import unittest

import numpy as np
from gtsam import Point3


def vector(*floats):
    """Create 3D double numpy array."""
    return np.array(floats, dtype=np.float)


def compose(*poses):
    """Compose all Pose3 transforms given as arguments from left to right."""
    return reduce((lambda x, y: x.compose(y)), poses)


def skew(a, b, c):
    """Return skew-symmetric matrix for vector a,b,c."""
    return np.array([[0, -c, b], [c, 0, -a], [-b, a, 0]], dtype=np.float)


def unit_twist(w, p):
    """Create unit twist for axis direction w, p some point on axis."""
    return np.hstack((w, np.cross(p, w)))


def hat(xi):
    """Return 4*4 se<3> element corresponding to twist coordinates xi."""
    S = np.zeros((4, 4), np.float)
    S[:3, :3] = skew(*xi[:3])
    S[:3, 3] = xi[3:]
    return S


def spatial_velocity(J, qdot, ps):
    """ Calculate the spatial velocity of a point ps.
        Keyword arguments:
            J -- spatial manipulator Jacobian
            qdot -- vector of joint velocities
            ps -- spatial coordinates of point.
    """
    return np.dot(hat(np.dot(J, qdot)), ps)


def vector_of_point3(p):
    """Convert Point3 to numpy array."""
    return vector(p.x(), p.y(), p.z())


def point3_of_vector(v):
    """Convert numpy array to Point3."""
    assert v.shape == (3,)
    return Point3(v[0], v[1], v[2])


class FetchTestCase(unittest.TestCase):
    """Base class with GTSAM assert utils."""

    def gtsamAssertEquals(self, actual, expected, tol=1e-2):
        """Helper function that prints out actual and expected if not equal."""
        equal = actual.equals(expected, tol)
        if not equal:
            raise self.failureException(
                "Values are not equal:\n{}!={}".format(actual, expected))


def plot_trajectory(trajectory, ax, **kwargs):
    """Plot trajectory using matplotlib scatter."""
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    X = np.array([g.x() for _t, g in trajectory])
    Y = np.array([g.y() for _t, g in trajectory])
    try:
        Z = np.array([g.z() for _t, g in trajectory])
    except AttributeError:
        Z = np.zeros_like(X)
    ax.scatter(X, Y, Z, **kwargs)

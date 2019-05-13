"""
Closed-form 6DOF elimination IK idea.
Requires sympy to be installed.
"""


import math
import os

import numpy as np
import sympy
from sympy import Matrix, N, Symbol, pprint, solve_poly_system
from sympy.solvers.solveset import nonlinsolve

from serial_link import SerialLink
from urdf_link import read_urdf
from utils import matrix_of_point3, vector


class FasterIK:
    """Do closed-form kinematics based on semi-group idea."""

    def __init__(self, robot: SerialLink):
        """ Construct from URDF file.
            Arguments:
                robot(SerialLink): serial link manipulator
        """
        self._robot = robot

    @classmethod
    def from_urdf(urdf_file: str, leaf_link_name: str):
        """Construct from URDF file."""
        link_dict = read_urdf(urdf_file)
        return FasterIK(SerialLink.from_urdf(link_dict, leaf_link_name))

    @staticmethod
    def Z(label: str = "", method="cs"):
        """ Create symbolic rotation matrix around Z-axis.
            Arguments:
                label(str): symbols created are "cL" and "sL", where L is label.
                method (str): "cs", "trig", or "cayley"
            Returns:
                (sympy.Matrix, unit constraint on "cL" and "sL")
        """
        if method == "trig":
            t = Symbol("t"+label, real=True)
            c, s = sympy.cos(t), sympy.sin(t)
            R = Matrix([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            return R, None, [t]
        if method == "cayley":
            w = Symbol("w"+label, real=True)
            K = 1 + w**2
            c, s = (1-w**2)/K, 2*w/K
            R = Matrix([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            return R, None, [w]
        if method == "cs":
            c = Symbol("c"+label, real=True)
            s = Symbol("s"+label, real=True)
            R = Matrix([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            return R, c**2 + s**2 - 1, [c, s]
        raise ValueError("unknwon method '{}'".format(method))

    @classmethod
    def rotation(cls, R12, R23, R3e, label: str = "", method="cs"):
        """ Calculate symbolic expression of the rotation matrix at the end of three links.
            Arguments:
                Rij: rotation matrices from frame i to frame j.
                     Frame 1 corresponding to joint angle 1 is assumed identity.
                     The end-effector frame E is related to frame 3 by R3e.
                label (str): symbols created are "cL1", "cL2" etc... where L is the label.
                method (str): "cs", "trig", or "cayley"
            Returns:
                rotation, [Z-matrices], [unit constraints for the angles], [symbols]
        """
        Z1, e1, syms1 = cls.Z(label+"1", method)
        Z2, e2, syms2 = cls.Z(label+"2", method)
        Z3, e3, syms3 = cls.Z(label+"3", method)
        syms = syms1 + syms2 + syms3
        constraints = [e for e in [e1, e2, e3] if e is not None]
        return Z1 * R12 * Z2 * R23 * Z3 * R3e, [Z1, Z2, Z3], constraints, syms

    @staticmethod
    def of_pose3(pose):
        """Convert Pose3 to numpy matrices, perhaps truncating."""
        R = pose.rotation().matrix()
        t = matrix_of_point3(pose.translation())
        Ri, ti = R.astype(np.int), t.astype(np.int)
        R = Ri if np.linalg.norm(R-Ri) < 1e-9 else R
        t = ti if np.linalg.norm(t-ti) < 1e-9 else t
        return R, t

    def pose(self, label: str = "", method="cs", offset=0):
        """ Calculate symbolic expression of the pose at the end of three links.
            Arguments:
                label(str): symbols created are "cL1", "cL2" etc... where L is the label.
                method (str): "cs", "trig", or "cayley"
                offset(int): 0 or 3, to select first three or last three joints
            Returns:
                rotation, translation, [unit constraints for the angles], [symbols]
        """
        frames = self._robot.link_transforms()
        R12, t12 = self.of_pose3(frames[0+offset])
        R23, t23 = self.of_pose3(frames[1+offset])
        R3e, t3e = self.of_pose3(frames[2+offset])

        Re, Zs, constraints, syms = FasterIK.rotation(
            R12, R23, R3e, label, method)
        Z1, Z2, Z3 = Zs
        te = Z1*(t12 + R12*Z2*(t23 + R23*Z3*t3e))
        return Re, te, Zs, constraints, syms

    @staticmethod
    def cs(c, s, theta):
        """Make subsitution for c/s parameters, given angle joint theta."""
        return [(c, math.cos(theta)), (s, math.sin(theta))]

    @staticmethod
    def pose2numpy(Re, te, substitutions):
        """Substitute and convert to numpy."""
        R = sympy.matrix2numpy(N(Re.subs(substitutions)), np.float)
        t = sympy.matrix2numpy(N(te.subs(substitutions)), np.float).flatten()
        return R, t

    @staticmethod
    def print_solutions(solutions):
        """Pretty print symbolics solutions from a solve."""
        for i, s in enumerate(solutions):
            print("Solution {}:\n".format(i+1))
            for si in sympy.cse(sympy.simplify(s)):
                pprint(si)
                print()

    @staticmethod
    def solutions_for(solutions, *substitutions):
        """Return numerical solutions given substitution list."""
        def f(solution):
            return np.array([N(s) for s in solution], np.float)

        return [f([si.subs([*substitutions]) for si in s]) for s in solutions]

    @classmethod
    def print_solutions_for(cls, solutions, *substitutions):
        """Pretty print numerical solutions given substitution list."""
        def f(c, s):
            return round(math.degrees(math.atan2(s, c)))

        def g(s):
            return [f(s[2*i], s[2*i+1]) for i in range(len(s)//2)]

        for s in cls.solutions_for(solutions, *substitutions):
            print([round(si, 3) for si in s])
            print(g(s))


class FasterIK_RRR(FasterIK):
    """Closed-form kinematics for 3-link arm."""

    def fk(self, q):
        """ Forward Kinematics for RRR arm.
            Arguments:
                robot(SerialLink): serial link manipulator
                q(vector): joint angles
            Returns:
                rotation, translation
        """
        Re, te, _, _, syms = super().pose()
        c1, s1, c2, s2, c3, s3 = syms
        substitutions = self.cs(
            c1, s1, q[0]) + self.cs(c2, s2, q[1]) + self.cs(c3, s3, q[2])
        return self.pose2numpy(Re, te, substitutions)

    def solve(self, lock_theta1=False, label: str = "", method="cs"):
        """ Solve for RRR link, as a function of position (x,0,z).
            The idea is that for non-zero y the first joint can be rotated appropriately.
            Typically returns two solutions, corresponding to two elbow configurations.
            Arguments:
                robot(SerialLink): serial link manipulator
                lock_theta1 (bool)
                label(str): symbols created are "cL1", "cL2" etc... where L is the label.
                method (str): "cs", "trig", or "cayley"
            Returns:
                [solutions], [symbols]
        """
        # Get a symbolic expression for te
        _, te, Zs, constraints, syms = super().pose(method=method)
        x, z = sympy.symbols('x z')
        td = Matrix([[x], [0], [z]], real=True)
        if method == "cs" and lock_theta1:
            # For some robots, all 5-D configurations beyond theta1 lie entirely in the
            # the y==0 plane and hence we can lock in theta=0.
            # Below we reduce the system to only solve for theta1 and theta2
            # This *also* disallows backward reaching from theta1=180
            c1, s1 = syms[0:2]
            syms = syms[2:]
            constraints = constraints[1:]
            te = te.subs([(c1, 1), (s1, 0)])
        equations = sympy.flatten(sympy.expand(4*(te-td))) + constraints
        if method == "cs":
            return solve_poly_system(equations, syms), syms + [x, z]
        else:
            return nonlinsolve(equations, syms), syms + [x, z]


class FasterIK_R6(FasterIK):
    """Closed-form kinematics for 6DOF manipulator."""

    def pose(self, method="cs"):
        """ Calculate symbolic expression of the pose at the end of 6DOF manipulator.
            Arguments:
                robot(SerialLink): serial link manipulator
                method (str): "cs", "trig", or "cayley"
            Returns:
                rotation, translation, [unit constraints for the angles], [symbols]
        """
        R4, t4, ZA, constraintsA, symsA = super().pose("a", method)
        R4e, t4e, ZB, constraintsB, symsB = super().pose(
            "b", method, offset=3)
        Re = R4 * R4e
        te = t4 + R4 * t4e
        return Re, te, ZA+ZB, constraintsA+constraintsB, symsA+symsB

    def fk(self, q):
        """ Forward Kinematics for 6DOF manipulator by evaluation symbolic expression.
            Arguments:
                robot(SerialLink): serial link manipulator
                q(vector): joint angles
            Returns:
                rotation, translation (numpy arrays)
        """
        Re, te, _, _, syms = self.pose()
        c1, s1, c2, s2, c3, s3, c4, s4, c5, s5, c6, s6 = syms
        substitutions = self.cs(c1, s1, q[0]) + self.cs(c2, s2, q[1]) + self.cs(
            c3, s3, q[2]) + self.cs(c4, s4, q[3]) + self.cs(c5, s5, q[4]) + self.cs(c6, s6, q[5])
        return self.pose2numpy(Re, te, substitutions)

    def solve(self, lock_theta1: bool = False, method="cs"):
        """ Solve for 6DOF arm, as a function of position (x,0,z).
            Arguments:
                robot(SerialLink): serial link manipulator
                lock_theta1 (bool): False by default
                method (str): "cs", "trig", or "cayley"
            Returns:
                [solutions], [symbols]
        """
        Re, te, Zs, constraints, syms = self.pose(method=method)
        x, z = sympy.symbols('x z')
        Rd = Matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        td = Matrix([[x], [0], [z]], real=True)
        if method == "cs" and lock_theta1:
            # Same as in solve
            c1, s1 = syms[0:2]
            syms = syms[2:]
            constraints = constraints[1:]
            Re = Re.subs([(c1, 1), (s1, 0)])
            te = te.subs([(c1, 1), (s1, 0)])
        equations = sympy.flatten(sympy.expand(
            64*(Re-Rd))) + sympy.flatten(sympy.expand(32*(te-td))) + constraints
        return solve_poly_system(equations, syms), syms + [x, z]

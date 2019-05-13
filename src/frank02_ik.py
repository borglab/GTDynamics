"""
Explore closed-form 6DOF elimination IK idea.
Requires sympy to be installed.
"""


import math
import os
import time
import unittest

import gtsam
import numpy as np
import sympy
from sympy import Matrix, N, Rational, S, Symbol, pprint, solve_poly_system
from sympy.solvers.solveset import nonlinsolve

from link_parameters import R6_calibration_dh, RRR_calibration_dh
from serial_link import SerialLink
from urdf_link import read_urdf
from utils import GtsamTestCase, matrix_of_point3, point3_of_vector, vector

MY_PATH = os.path.dirname(os.path.realpath(__file__))
URDFS_PATH = os.path.join(MY_PATH, '../urdfs')
sympy.init_printing()


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
    def RRR_rotation(cls, R12, R23, R3e, label: str = "", method="cs"):
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

    def RRR_pose(self, label: str = "", method="cs", offset=0):
        """ Calculate symbolic expression of the pose at the end of three links.
            Arguments:
                robot(SerialLink): serial link manipulator
                label(str): symbols created are "cL1", "cL2" etc... where L is the label.
                method (str): "cs", "trig", or "cayley"
                offset(int): 0 or 3, to select first three or last three joints
            Returns:
                rotation, translation, [unit constraints for the angles], [symbols]
        """
        def of_pose(pose):
            R = pose.rotation().matrix()
            t = matrix_of_point3(pose.translation())
            Ri, ti = R.astype(np.int), t.astype(np.int)
            R = Ri if np.linalg.norm(R-Ri) < 1e-9 else R
            t = ti if np.linalg.norm(t-ti) < 1e-9 else t
            return R, t

        frames = self._robot.link_transforms()
        R12, t12 = of_pose(frames[0+offset])
        R23, t23 = of_pose(frames[1+offset])
        R3e, t3e = of_pose(frames[2+offset])

        Re, Zs, constraints, syms = FasterIK.RRR_rotation(
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

    def RRR_fk(self, q):
        """ Forward Kinematics for RRR arm.
            Arguments:
                robot(SerialLink): serial link manipulator
                q(vector): joint angles
            Returns:
                rotation, translation
        """
        Re, te, _, _, syms = self.RRR_pose()
        c1, s1, c2, s2, c3, s3 = syms
        substitutions = self.cs(
            c1, s1, q[0]) + self.cs(c2, s2, q[1]) + self.cs(c3, s3, q[2])
        return self.pose2numpy(Re, te, substitutions)

    def RRR_solve(self, lock_theta1=False, label: str = "", method="cs"):
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
        _, te, Zs, constraints, syms = self.RRR_pose(method=method)
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
        pprint(equations)
        if method == "cs":
            return solve_poly_system(equations, syms), syms + [x, z]
        else:
            return nonlinsolve(equations, syms), syms + [x, z]

    def R6_pose(self, method="cs"):
        """ Calculate symbolic expression of the pose at the end of 6DOF manipulator.
            Arguments:
                robot(SerialLink): serial link manipulator
                method (str): "cs", "trig", or "cayley"
            Returns:
                rotation, translation, [unit constraints for the angles], [symbols]
        """
        R4, t4, ZA, constraintsA, symsA = self.RRR_pose("a", method)
        R4e, t4e, ZB, constraintsB, symsB = self.RRR_pose(
            "b", method, offset=3)
        Re = R4 * R4e
        te = t4 + R4 * t4e
        return Re, te, ZA+ZB, constraintsA+constraintsB, symsA+symsB

    def R6_fk(self, q):
        """ Forward Kinematics for 6DOF manipulator by evaluation symbolic expression.
            Arguments:
                robot(SerialLink): serial link manipulator
                q(vector): joint angles
            Returns:
                rotation, translation (numpy arrays)
        """
        Re, te, _, _, syms = self.R6_pose()
        c1, s1, c2, s2, c3, s3, c4, s4, c5, s5, c6, s6 = syms
        substitutions = self.cs(c1, s1, q[0]) + self.cs(c2, s2, q[1]) + self.cs(
            c3, s3, q[2]) + self.cs(c4, s4, q[3]) + self.cs(c5, s5, q[4]) + self.cs(c6, s6, q[5])
        return self.pose2numpy(Re, te, substitutions)

    def R6_solve(self, lock_theta1: bool = False, method="cs"):
        """ Solve for 6DOF arm, as a function of position (x,0,z).
            Arguments:
                robot(SerialLink): serial link manipulator
                lock_theta1 (bool): False by default
                method (str): "cs", "trig", or "cayley"
            Returns:
                [solutions], [symbols]
        """
        Re, te, Zs, constraints, syms = self.R6_pose(method=method)
        x, z = sympy.symbols('x z')
        Rd = Matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        td = Matrix([[x], [0], [z]], real=True)
        if method == "cs" and lock_theta1:
            # Same as in RRR_solve
            c1, s1 = syms[0:2]
            syms = syms[2:]
            constraints = constraints[1:]
            Re = Re.subs([(c1, 1), (s1, 0)])
            te = te.subs([(c1, 1), (s1, 0)])
        equations = sympy.flatten(sympy.expand(
            64*(Re-Rd))) + sympy.flatten(sympy.expand(32*(te-td))) + constraints
        # equations = [sympy.expand(e) for e in equations]
        pprint(equations)
        return solve_poly_system(equations, syms), syms + [x, z]

    @staticmethod
    def print_solutions(solutions):
        """Pretty print symbolics solutions from a solve."""
        for i, s in enumerate(solutions):
            print("Solution {}:\n".format(i+1))
            for si in sympy.cse(sympy.simplify(s)):
                pprint(si)
                print()

    @staticmethod
    def print_solutions_for(solutions, *substitutions):
        """Pretty print numerical solutions given substitution list."""
        def f(c, s):
            return round(math.degrees(math.atan2(N(s), N(c))))

        def g(s):
            return [f(s[2*i], s[2*i+1]) for i in range(len(s)//2)]

        for s in solutions:
            s = [si.subs([*substitutions]) for si in s]
            print([round(si, 3) for si in s])
            print(g(s))

    def RA(self):
        """Expression for rotation matrix at end of first three links."""
        # TODO(frank): calculate R12
        I3 = Matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        return self.ik.RRR_rotation(I3, I3, I3, "A")

    def calculate_theta_A(self, R4, t4):
        """Calculate joint angles thetaA = (theta_1, theta_2, theta_3) from the intermediate pose (R4,t4)."""
        M4 = Matrix(R4.matrix())
        return vector(0, 0, 0)


O = np.array([[0], [0], [0]])
X = np.array([[1], [0], [0]])
Y = np.array([[0], [1], [0]])
Z = np.array([[0], [0], [1]])

I3 = np.column_stack([X, Y, Z])


def pose(R, t):
    """Create a Pose3 out of 2 numpy arrays."""
    return gtsam.Pose3(gtsam.Rot3(R), point3_of_vector(t.flatten()))


class TestFasterIKRRR(GtsamTestCase):
    """Unit tests for FasterIK class, using RRR elbow arm."""

    def setUp(self):
        """Read URDF from file and create class instance."""
        self.elbow = SerialLink(RRR_calibration_dh)
        self.ik = FasterIK(self.elbow)

    def test_RRR_rotation(self):
        """Check expression for rotation matrix at end of three links."""
        R12 = np.column_stack([X, Z, -Y])
        R23 = np.column_stack([X, Y, Z])
        R34 = np.column_stack([X, -Z, Y])
        R3 = [R12, R23, R34]
        Re, Zs, constraints, syms = FasterIK.RRR_rotation(*R3)
        self.assertIsInstance(Re, sympy.Matrix)
        self.assertIsInstance(constraints, list)
        self.assertEqual(len(constraints), 3)
        self.assertEqual(len(syms), 6)

    def test_RRR_pose(self):
        """Check expression for pose at end of three links."""
        Re, te, _, constraints, syms = self.ik.RRR_pose()
        self.assertIsInstance(Re, sympy.Matrix)
        self.assertIsInstance(constraints, list)
        self.assertEqual(len(constraints), 3)
        self.assertEqual(len(syms), 6)

    def test_RRR_fk(self):
        """Check forward kinematics for two configurations of RRR."""
        R, t = self.ik.RRR_fk(vector(0, 0, 0))  # rest
        np.testing.assert_array_almost_equal(t, vector(2, 0, 1))
        np.testing.assert_array_almost_equal(R, np.column_stack([X, Y, Z]))

        R, t = self.ik.RRR_fk(vector(0, math.pi/2, 0))  # up
        np.testing.assert_array_almost_equal(t, vector(0, 0, 3))
        np.testing.assert_array_almost_equal(R, np.column_stack([Z, Y, -X]))

    @unittest.skip("too slow")
    def test_RRR_solve(self):
        solutions, syms = self.ik.RRR_solve(lock_theta1=True)
        self.assertEqual(len(solutions), 2)
        self.assertEqual(len(syms), 6)


class TestFasterIKR6(GtsamTestCase):
    """Unit tests for FasterIK class, using 6DOF manipulator."""

    def setUp(self):
        """Read URDF from file and create class instance."""
        self.r6 = SerialLink(R6_calibration_dh)
        self.ik = FasterIK(self.r6)

    def test_R6_pose(self):
        """Check expression for pose at end of 6DOF manipulator."""
        Re, te, Zs, constraints, syms = self.ik.R6_pose()
        self.assertIsInstance(Re, sympy.Matrix)
        self.assertIsInstance(constraints, list)
        self.assertEqual(len(constraints), 6)
        self.assertEqual(len(syms), 12)

    def test_R6_fk(self):
        """Check forward kinematics for two configurations of 6DOF manipulator."""
        R, t = self.ik.R6_fk(vector(0, 0, 0, 0, 0, 0))  # rest
        np.testing.assert_array_almost_equal(t, vector(2, 0, 2))
        np.testing.assert_array_almost_equal(R, np.column_stack([X, Y, Z]))

        R, t = self.ik.R6_fk(vector(0, math.pi/2, 0, 0, 0, 0))  # up
        np.testing.assert_array_almost_equal(t, vector(-1, 0, 3))
        np.testing.assert_array_almost_equal(R, np.column_stack([Z, Y, -X]))

    @unittest.skip("too slow")
    def test_R6_solve(self):
        solutions, syms = self.ik.R6_solve(True)
        self.assertEqual(len(solutions), 2)
        self.assertEqual(len(syms), 6)


class TestFasterIKFanuc(GtsamTestCase):
    """Unit tests for FasterIK class, using RRR elbow arm."""

    def setUp(self):
        """Read URDF from file and create class instance."""
        urdf_file = os.path.join(URDFS_PATH, "fanuc_lrmate200id.urdf")
        self.ik = self.ik.from_urdf(urdf_file, "Part6")


def timeit(method):
    """https://medium.com/pythonhive/python-decorator-to-measure-the-execution-time-of-methods-fa04cb6bb36d"""
    def timed(*args, **kw):
        ts = time.time()
        result = method(*args, **kw)
        te = time.time()
        if 'log_time' in kw:
            name = kw.get('log_name', method.__name__.upper())
            kw['log_time'][name] = int((te - ts) * 1000)
        else:
            print('%r  %2.2f ms' % (method.__name__, (te - ts) * 1000))
        return result
    return timed


@timeit
def run_rrr():
    """Run RRR solver for simple example."""
    ik = FasterIK(SerialLink(RRR_calibration_dh))
    solutions, syms = ik.RRR_solve(lock_theta1=True, method="cs")
    if solutions is None:
        print("no solution!")
    else:
        FasterIK.print_solutions(solutions)
        x, z = syms[-2:]
        FasterIK.print_solutions_for(solutions, (x, 1.5), (z, 2))


@timeit
def run_r6():
    """Run 6DOF solver for simple example."""
    ik = FasterIK(SerialLink(R6_calibration_dh))
    solutions, syms = ik.R6_solve(lock_theta1=True)
    if solutions is None:
        print("no solution!")
    else:
        FasterIK.print_solutions(solutions)
        x, z = syms[-2:]
        FasterIK.print_solutions_for(solutions, (x, 1.5), (z, 2))


if __name__ == "__main__":
    # run_rrr()
    # run_r6()
    unittest.main()

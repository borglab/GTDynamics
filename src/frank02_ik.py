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
from sympy import Matrix, N, S, Symbol, pprint, solve_poly_system, solveset
from sympy.solvers.solveset import nonlinsolve

from serial_link import SerialLink
from urdf_link import read_urdf
from utils import GtsamTestCase, vector

MY_PATH = os.path.dirname(os.path.realpath(__file__))
URDFS_PATH = os.path.join(MY_PATH, '../urdfs')
sympy.init_printing()


class FasterIK:
    """Do closed-form kinematics based on semi-group idea."""

    def __init__(self, urdf_file):
        """Construct from URDF file."""
        link_dict = read_urdf(urdf_file)
        self._robot = SerialLink.from_urdf(link_dict, leaf_link_name="Part6")

    @staticmethod
    def Z(label: str = "", trig=False):
        """ Create symbolic rotation matrix around Z-axis.
            Arguments:
                label(str): symbols created are "cL" and "sL", where L is label.
                trig (bool): whether to use cos/sin rather than c,s,c^2+s^2==1
            Returns:
                (sympy.Matrix, unit constraint on "cL" and "sL")
        """
        if trig:
            theta = Symbol("\\theta"+label, real=True)
            c, s = sympy.cos(theta), sympy.sin(theta)
            R = Matrix([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            return R, [], [theta]
        else:
            c = Symbol("c"+label, real=True)
            s = Symbol("s"+label, real=True)
            R = Matrix([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            return R, c**2 + s**2 - 1, [c, s]

    @classmethod
    def RRR_rotation(cls, R12, R23, R3e, label: str = "", trig=False):
        """ Calculate symbolic expression of the rotation matrix at the end of three links.
            Arguments:
                Rij: rotation matrices from frame i to frame j.
                     Frame 1 corresponding to joint angle 1 is assumed identity.
                     The end-effector frame E is related to frame 3 by R3e.
                label (str): symbols created are "cL1", "cL2" etc... where L is the label.
                trig (bool): whether to use cos/sin rather than c,s,c^2+s^2==1
            Returns:
                rotation, [Z-matrices], [unit constraints for the angles], [symbols]
        """
        Z1, e1, syms1 = cls.Z(label+"1", trig)
        Z2, e2, syms2 = cls.Z(label+"2", trig)
        Z3, e3, syms3 = cls.Z(label+"3", trig)
        syms = syms1 + syms2 + syms3
        return Z1 * R12 * Z2 * R23 * Z3 * R3e, [Z1, Z2, Z3], [e1, e2, e3], syms

    @classmethod
    def RRR_pose(cls, R12, t12, R23, t23, R3e, t3e, label: str = "", trig=False):
        """ Calculate symbolic expression of the pose at the end of three links.
            Arguments:
                Tij: link transform from frame i to frame j.
                     Frame 1 corresponding to joint angle 1 is assumed identity.
                     The end-effector frame E is related to frame 3 by T3e.
                label(str): symbols created are "cL1", "cL2" etc... where L is the label.
            Returns:
                rotation, translation, [unit constraints for the angles], [symbols]
        """
        Re, Zs, constraints, syms = cls.RRR_rotation(
            R12, R23, R3e, label, trig)
        Z1, Z2, Z3 = Zs
        te = Z1*(t12 + R12*Z2*(t23 + R23*Z3*t3e))
        return Re, te, Zs, constraints, syms

    @classmethod
    def R6_pose(cls, *robot):
        """ Calculate symbolic expression of the pose at the end of 6DOF manipulator.
            Arguments:
                robot: list of [R, t, R, t...]
            Returns:
                rotation, translation, [unit constraints for the angles], [symbols]
        """
        R4, t4, ZA, constraintsA, symsA = cls.RRR_pose(*robot[:6], "a")
        R4e, t4e, ZB, constraintsB, symsB = cls.RRR_pose(*robot[6:], "b")
        Re = R4 * R4e
        te = t4 + R4 * t4e
        return Re, te, ZA+ZB, constraintsA+constraintsB, symsA+symsB

    @classmethod
    def RRR_solve(cls, R12, t12, R23, t23, R3e, t3e, lock_theta1=False, label: str = ""):
        """ Solve for RRR link, as a function of position (x,0,z).
            The idea is that for non-zero y the first joint can be rotated appropriately.
            Typically returns two solutions, corresponding to two elbow configurations.
        """
        # Get a symbolic expression for te
        _, te, Zs, constraints, syms = cls.RRR_pose(
            R12, t12, R23, t23, R3e, t3e)
        x, z = sympy.symbols('x z')
        td = Matrix([[x], [0], [z]], real=True)
        if lock_theta1:
            # For some robots, all 5-D configurations beyond theta1 lie entirely in the
            # the y==0 plane and hence we can lock in theta=0.
            # Below we reduce the system to only solve for theta1 and theta2
            # This *also* disallows backward reaching from theta1=180
            c1, s1 = syms[0:2]
            syms = syms[2:]
            constraints = constraints[1:]
            te = te.subs([(c1, 1), (s1, 0)])
        equations = sympy.flatten(te-td) + constraints
        return solve_poly_system(equations, syms), syms + [x, z]

    @classmethod
    def R6_solve(cls, lock_theta1, *robot):
        """ Solve for 6DOF arm, as a function of position (x,0,z).
        """
        Re, te, Zs, constraints, syms = cls.R6_pose(*robot)
        x, z = sympy.symbols('x z')
        Rd = Matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        td = Matrix([[x], [0], [z]], real=True)
        if lock_theta1:
            # Same as in RRR_solve
            c1, s1 = syms[0:2]
            syms = syms[2:]
            constraints = constraints[1:]
            te = te.subs([(c1, 1), (s1, 0)])
        equations = sympy.flatten(Re-Rd) + sympy.flatten(te-td) + constraints
        # equations = [sympy.expand(e) for e in equations]
        print(equations)
        return solve_poly_system(equations, syms), syms + [x, z]

    @staticmethod
    def print_solutions(solutions):
        """Pretty print symbolics solutions from a solve."""
        print(solutions)
        for s in solutions:
            print("---------------------------")
            for si in sympy.cse(sympy.simplify(s)):
                pprint(si)

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
        return FasterIK.RRR_rotation(I3, I3, I3, "A")

    def calculate_theta_A(self, R4, t4):
        """Calculate joint angles thetaA = (theta_1, theta_2, theta_3) from the intermediate pose (R4,t4)."""
        M4 = Matrix(R4.matrix())
        return vector(0, 0, 0)


class TestInverseKinematics(GtsamTestCase):
    """Unit tests for FasterIK class."""

    R12 = Matrix([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    I3 = Matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    t12 = Matrix([[0], [0], [1]])
    t23 = Matrix([[0], [1], [0]])
    t3e = t23
    ROBOT = [R12, t12, I3, t23, I3, t3e]

    def setUp(self):
        """Read URDF from file and create class instance."""
        urdf_file = os.path.join(URDFS_PATH, "fanuc_lrmate200id.urdf")
        self.ik = FasterIK(urdf_file)

    def test_RRR_rotation(self):
        """Check expression for rotation matrix at end of three links."""
        R12 = Matrix([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
        I3 = Matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        Re, Zs, constraints, syms = FasterIK.RRR_rotation(R12, I3, I3)
        self.assertIsInstance(Re, sympy.Matrix)
        self.assertIsInstance(constraints, list)
        self.assertEqual(len(constraints), 3)
        self.assertEqual(len(syms), 6)

    def test_RRR_pose(self):
        """Check expression for pose at end of three links."""
        Re, te, Zs, constraints, syms = FasterIK.RRR_pose(*self.ROBOT)
        self.assertIsInstance(Re, sympy.Matrix)
        self.assertIsInstance(constraints, list)
        self.assertEqual(len(constraints), 3)
        self.assertEqual(len(syms), 6)

    def test_R6_pose(self):
        """Check expression for pose at end of 6DOF manipulator."""
        robot = 2*self.ROBOT
        Re, te, Zs, constraints, syms = FasterIK.R6_pose(*robot)
        self.assertIsInstance(Re, sympy.Matrix)
        self.assertIsInstance(constraints, list)
        self.assertEqual(len(constraints), 6)
        self.assertEqual(len(syms), 12)

    @unittest.skip("too slow")
    def test_RRR_solve(self):
        solutions, syms = FasterIK.RRR_solve(*self.ROBOT, lock_theta1=True)
        self.assertEqual(len(solutions), 2)
        self.assertEqual(len(syms), 6)

    @unittest.skip("too slow")
    def test_R6_solve(self):
        robot = 2*self.ROBOT
        solutions, syms = FasterIK.R6_solve(True, *robot)
        self.assertEqual(len(solutions), 2)
        self.assertEqual(len(syms), 6)

    def test_RA(self):
        """Check expression for rotation matrix at end of first three links."""
        Re, Zs, constraints, syms = self.ik.RA()
        self.assertIsInstance(Re, sympy.Matrix)

    def test_calculate_theta_A(self):
        """Test all."""
        R4 = gtsam.Rot3()
        t4 = gtsam.Point3(1, 2, 3)
        theta_A = self.ik.calculate_theta_A(R4, t4)
        # np.testing.assert_array_almost_equal(theta_A, vector(1, 2, 3))


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
    robot = TestInverseKinematics.ROBOT
    solutions, syms = FasterIK.RRR_solve(*robot, lock_theta1=True)
    FasterIK.print_solutions(solutions)
    x, z = syms[-2:]
    FasterIK.print_solutions_for(solutions, (x, 1.5), (z, 2))


@timeit
def run_r6():
    """Run 6DOF solver for simple example."""
    robot = 2*TestInverseKinematics.ROBOT
    lock_theta1 = False
    solutions, syms = FasterIK.R6_solve(lock_theta1, *robot)
    FasterIK.print_solutions(solutions)
    x, z = syms[-2:]
    FasterIK.print_solutions_for(solutions, (x, 1.5), (z, 2))


if __name__ == "__main__":
    run_rrr()
    # run_r6()
    unittest.main()

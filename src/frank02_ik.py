"""
Explore closed-form 6DOF elimination IK idea.
Requires sympy to be installed.
"""


import os
import time

import sympy

from faster_ik import FasterIK, FasterIK_R6, FasterIK_RRR
from link_parameters import R6_calibration_dh, RRR_calibration_dh
from serial_link import SerialLink
from urdf_link import read_urdf

MY_PATH = os.path.dirname(os.path.realpath(__file__))
URDFS_PATH = os.path.join(MY_PATH, '../urdfs')
sympy.init_printing()


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
    ik = FasterIK_RRR(SerialLink(RRR_calibration_dh))
    solutions, syms = ik.solve(lock_theta1=True, method="cs")
    if solutions is None:
        print("no solution!")
    else:
        FasterIK.print_solutions(solutions)
        x, z = syms[-2:]
        FasterIK.print_solutions_for(solutions, (x, 1.5), (z, 2))


@timeit
def run_r6():
    """Run 6DOF solver for simple example."""
    ik = FasterIK_R6(SerialLink(R6_calibration_dh))
    solutions, syms = ik.solve(lock_theta1=True)
    if solutions is None:
        print("no solution!")
    else:
        FasterIK.print_solutions(solutions)
        x, z = syms[-2:]
        FasterIK.print_solutions_for(solutions, (x, 1.5), (z, 2))


if __name__ == "__main__":
    run_rrr()
    run_r6()
    unittest.main()

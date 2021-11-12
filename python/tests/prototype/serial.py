"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  serial.py
 * @brief Ptototype Serial class for serial manipulators.
 * @author Frank Dellaert
"""

from typing import Optional, Tuple

import gtdynamics as gtd
import numpy as np
from gtsam import Pose3


def compose(A: Tuple[Pose3, np.ndarray], B: Tuple[Pose3, np.ndarray]):
    """Monoid operation for pose,Jacobian pairs."""
    aTb, Jb = A
    bTc, Jc = B
    assert Jb.shape[0] == 6 and Jc.shape[0] == 6, f"{Jb.shape} and {Jc.shape}"

    # Compose poses
    aTc = aTb.compose(bTc)

    # Check if one of specs is an offset:
    if Jb.shape[1] == 0:
        return aTc, Jc

    c_Ad_b = bTc.inverse().AdjointMap()
    if Jc.shape[1] == 0:
        return aTc, c_Ad_b @ Jb

    # if not, do normal case:
    return aTc, np.hstack((c_Ad_b @ Jb, Jc))


class Serial():
    """Three-link arm class."""

    def __init__(self, sMb, axes: np.ndarray):
        """Create from end-effector at rest and Jacobian.

        Arguments:
            sMb: rest pose of "body" with respect to "spatial" frame
            axes: screw axes of all joints expressed in body frame
        """
        assert isinstance(sMb, Pose3)
        assert isinstance(axes, np.ndarray)
        self.sMb = sMb
        self.axes = np.expand_dims(axes, 1) if len(axes.shape) == 1 else axes

    @classmethod
    def compose(cls, *components):
        """Create from a variable number of other Serial instances."""
        spec = components[0].spec()
        for component in components[1:]:
            spec = compose(spec, component.spec())
        return cls(*spec)

    def spec(self):
        """Return end-effector at rest and Jacobian."""
        return self.sMb, self.axes

    def __repr__(self):
        return f"Serial\n: {self.sMb}\n{np.round(self.axes,3)}\n"

    @classmethod
    def from_robot(cls, robot: gtd.Robot,
                   base_name: Optional[str] = None,
                   joint_range: Optional[Tuple[int, int]] = None):
        """Initialize from a robot.

        Arguments:
            robot: a GTDynamics Robot instance
            base_name: add offset for base link, if given
            joint_range: a range of joint indices (base 0)
        """
        # Get desired joints from robot instance.
        if joint_range is None:
            joint_range = 0, robot.numJoints()
        joints = robot.joints()[joint_range[0]:joint_range[1]]

        # Convert all joints into pose/Jacobian pairs.
        pairs = [cls(joint.pMc(), joint.cScrewAxis()) for joint in joints]

        if base_name is not None:
            # Create offset to first link parent.
            assert joint_range is None or joint_range[0] == 0, \
                "Cannot have base name if first joint is not 0"
            base_link = robot.link(base_name)
            sM0 = base_link.bMcom()
            offset = Serial(sM0, np.zeros((6, 0)))
            pairs = [offset] + pairs

        # Now, let compose do the work!
        return cls.compose(*pairs)

    def poe(self, q: np.ndarray,
            fTe: Optional[Pose3] = None,
            J: Optional[np.ndarray] = None):
        """ Perform forward kinematics given q, return Pose of end-effector.
            When q is smaller than #joints, actuates last joints in chain.

        Arguments:
            q (np.ndarray): joint angles for all joints.
            fTe: optionally, the end-effector pose with respect to final link.
            J: optionally, the manipulator Jacobian.
        Returns:
            jTe (Pose3)
        """
        # Check input.
        n = len(q)
        A = self.axes
        assert n == A.shape[1]

        # Calculate exponentials.
        exp = [Pose3.Expmap(A[:, j] * q[j]) for j in range(n)]

        if J is None:
            # Just do product.
            poe = self.sMb
            for T_j in exp:
                poe = poe.compose(T_j)
            return poe if fTe is None else poe.compose(fTe)
        else:
            # Compute FK + Jacobian with monoid compose.
            assert J.shape == (6, len(q)), f"Needs 6x{len(q)} J."
            Empty = np.zeros((6, 0))
            pair = self.sMb, Empty
            for j in range(n):
                pair = compose(pair, (exp[j], np.expand_dims(A[:, j], 1)))
            if fTe is not None:
                pair = compose(pair, (fTe, Empty))  # Adjoints Jacobian to E!
            poe, J[:, :] = pair
            return poe

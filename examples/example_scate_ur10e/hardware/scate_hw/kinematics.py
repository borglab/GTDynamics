"""Forward kinematics of a live-app robot description, and wall/floor clearance.

The description (scate_live describe, robots/ur10e.json) holds the link
frames at rest and the joints' space-frame screw axes; links are posed by
the product of exponentials T_l(q) = exp(S_1 q_1) ... exp(S_l q_l) T_l(0),
exactly as apps/scate/shared/kinematics.js does. Matrices are 4x4 nested
lists. No numpy, so it runs anywhere.
"""

from __future__ import annotations

import json
import math


def load_description(path):
    with open(path) as file:
        robot = json.load(file)
    if robot.get('format') != 'scate-live-robot':
        raise ValueError(f'{path} is not a scate_live robot description')
    return robot


def identity():
    return [[1.0 if r == c else 0.0 for c in range(4)] for r in range(4)]


def multiply(a, b):
    return [[sum(a[r][k] * b[k][c] for k in range(4)) for c in range(4)] for r in range(4)]


def pose_matrix(pose):
    """Transform of an [x, y, z, qx, qy, qz, qw] pose."""
    x, y, z, qx, qy, qz, qw = pose
    return [
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw), x],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw), y],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy), z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def screw_exp(screw, theta):
    """exp of the revolute twist [w; v] * theta (|w| = 1)."""
    wx, wy, wz, vx, vy, vz = screw
    s, c = math.sin(theta), 1 - math.cos(theta)
    W = [[0, -wz, wy], [wz, 0, -wx], [-wy, wx, 0]]
    W2 = [[sum(W[r][k] * W[k][col] for k in range(3)) for col in range(3)] for r in range(3)]
    R = [[(1 if r == col else 0) + s * W[r][col] + c * W2[r][col] for col in range(3)] for r in range(3)]
    G = [[(theta if r == col else 0) + c * W[r][col] + (theta - s) * W2[r][col] for col in range(3)]
         for r in range(3)]
    t = [G[r][0] * vx + G[r][1] * vy + G[r][2] * vz for r in range(3)]
    return [R[0] + [t[0]], R[1] + [t[1]], R[2] + [t[2]], [0.0, 0.0, 0.0, 1.0]]


def link_transforms(robot, q):
    """World transform of every link (link 0 the base) at joint angles q."""
    rest = robot.setdefault('_rest', [pose_matrix(link['rest']) for link in robot['links']])
    out = [rest[0]]
    motion = identity()
    for j, joint in enumerate(robot['joints']):
        motion = multiply(motion, screw_exp(joint['screw'], q[j]))
        out.append(multiply(motion, rest[j + 1]))
    return out


def transform_point(m, p):
    return [m[r][0] * p[0] + m[r][1] * p[1] + m[r][2] * p[2] + m[r][3] for r in range(3)]


def sphere_centers(robot, q):
    """[(center, radius)] of the collision spheres at q."""
    transforms = link_transforms(robot, q)
    return [(transform_point(transforms[s['link']], s['center']), s['radius']) for s in robot['spheres']]


def plane_distance(plane, p):
    n = plane['normal']
    return n[0] * p[0] + n[1] * p[1] + n[2] * p[2] - plane['offset']


def plane_clearance(robot, q, planes):
    """(gap, plane name): smallest gap of any sphere surface to the planes; (inf, None) with none."""
    best, name = math.inf, None
    if not planes:
        return best, name
    for center, radius in sphere_centers(robot, q):
        for plane in planes:
            gap = plane_distance(plane, center) - radius
            if gap < best:
                best, name = gap, plane['name']
    return best, name

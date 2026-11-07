#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------

## @file rrt.py
# @brief RRT-Connect path seeding for GPMP2 trajectory initialization.
# @author GTDynamics contributors

"""RRT-Connect in a normalized configuration box, used to seed GPMP2 states.

A feasible path is found with two trees grown from the start and the goal, then
resampled by arc length to the planner's support states, with velocities by
central differences and zero at both ends. This mirrors the BAR Lab paper
planner's initialization.
"""

from __future__ import annotations

import numpy as np


class Tree:
    """Configurations with parent back-pointers."""

    def __init__(self, root: np.ndarray):
        self.nodes = [np.asarray(root, dtype=float)]
        self.parents = [-1]

    def add(self, q: np.ndarray, parent: int) -> int:
        """Append a configuration and return its index."""
        self.nodes.append(np.asarray(q, dtype=float))
        self.parents.append(parent)
        return len(self.nodes) - 1

    def nearest(self, q: np.ndarray, span: np.ndarray) -> int:
        """Return the index of the node closest to ``q`` in the normalized metric."""
        distances = np.linalg.norm((np.asarray(self.nodes) - q) / span, axis=1)
        return int(np.argmin(distances))

    def path_to(self, index: int) -> list[np.ndarray]:
        """Return configurations from the root to ``index``."""
        path = []
        while index != -1:
            path.append(self.nodes[index])
            index = self.parents[index]
        return path[::-1]


def steer(q_from, q_to, step, span):
    """Step at most ``step`` (normalized) from ``q_from`` toward ``q_to``."""
    delta = (q_to - q_from) / span
    distance = np.linalg.norm(delta)
    if distance <= step:
        return np.asarray(q_to, dtype=float), True
    return q_from + (step / distance) * delta * span, False


def edge_clear(q_a, q_b, collision_free, span, resolution) -> bool:
    """Whether interior samples of the segment ``q_a`` to ``q_b`` are collision free."""
    distance = np.linalg.norm((q_b - q_a) / span)
    steps = int(np.ceil(distance / resolution))
    return all(collision_free(q_a + (i / steps) * (q_b - q_a)) for i in range(1, steps))


def extend(tree, q_target, collision_free, step, span, resolution):
    """Grow ``tree`` one step toward ``q_target``; return (status, new index)."""
    near = tree.nearest(q_target, span)
    q_new, reached = steer(tree.nodes[near], q_target, step, span)
    if not collision_free(q_new) or not edge_clear(
        tree.nodes[near], q_new, collision_free, span, resolution
    ):
        return "trapped", -1
    return ("reached" if reached else "advanced"), tree.add(q_new, near)


def connect(tree, q_target, collision_free, step, span, resolution):
    """Extend ``tree`` toward ``q_target`` until it is reached or stalls."""
    last = -1
    while True:
        status, index = extend(tree, q_target, collision_free, step, span, resolution)
        if status == "advanced":
            last = index
            continue
        return ("reached", index) if status == "reached" else ("trapped", last)


def rrt_connect(
    q_start,
    q_goal,
    lower,
    upper,
    collision_free,
    step=0.05,
    resolution=0.02,
    max_iterations=5000,
    seed=0,
    endpoint_collision_free=None,
) -> list[np.ndarray]:
    """Return a feasible path from ``q_start`` to ``q_goal`` by bidirectional tree growth."""
    q_start, q_goal = np.asarray(q_start, dtype=float), np.asarray(q_goal, dtype=float)
    lower, upper = np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)
    span = np.where((upper - lower) > 0, upper - lower, 1.0)
    endpoint_check = endpoint_collision_free or collision_free
    if not endpoint_check(q_start):
        raise ValueError("RRT-Connect start configuration is in collision")
    if not endpoint_check(q_goal):
        raise ValueError("RRT-Connect goal configuration is in collision")
    rng = np.random.default_rng(seed)
    tree_a, tree_b = Tree(q_start), Tree(q_goal)
    start_side = True
    for _ in range(max_iterations):
        sample = lower + rng.random(len(lower)) * (upper - lower)
        status, new = extend(tree_a, sample, collision_free, step, span, resolution)
        if status != "trapped":
            outcome, joined = connect(
                tree_b, tree_a.nodes[new], collision_free, step, span, resolution
            )
            if outcome == "reached":
                path_a, path_b = tree_a.path_to(new), tree_b.path_to(joined)
                return path_a + path_b[::-1] if start_side else path_b + path_a[::-1]
        tree_a, tree_b = tree_b, tree_a
        start_side = not start_side
    raise ValueError(f"RRT-Connect found no path in {max_iterations} iterations")


def shortcut(
    path, collision_free, span, resolution, seed=0, tries=300
) -> list[np.ndarray]:
    """Return ``path`` with random pairs of waypoints joined wherever the straight edge is free."""
    rng = np.random.default_rng(seed)
    points = [np.asarray(p, dtype=float) for p in path]
    for _ in range(tries):
        if len(points) < 3:
            break
        i, j = sorted(rng.choice(len(points), size=2, replace=False))
        if j - i > 1 and edge_clear(
            points[i], points[j], collision_free, span, resolution
        ):
            points = points[: i + 1] + points[j:]
    return points


def resample_path(path, state_count, span) -> list[np.ndarray]:
    """Resample a waypoint path to ``state_count`` states evenly spaced by arc length."""
    points = [np.asarray(p, dtype=float) for p in path]
    segments = [
        np.linalg.norm((points[i + 1] - points[i]) / span)
        for i in range(len(points) - 1)
    ]
    cumulative = np.concatenate(([0.0], np.cumsum(segments)))
    total = cumulative[-1]
    if total == 0.0:
        return [points[0].copy() for _ in range(state_count)]
    states = []
    for k in range(state_count):
        target = total * k / (state_count - 1)
        j = min(
            max(int(np.searchsorted(cumulative, target, side="right") - 1), 0),
            len(points) - 2,
        )
        local = (target - cumulative[j]) / segments[j] if segments[j] > 0 else 0.0
        states.append(points[j] + local * (points[j + 1] - points[j]))
    return states


def central_velocities(line, total_time) -> list[np.ndarray]:
    """Return central-difference velocities of evenly timed states, zero at both ends."""
    delta_t = total_time / (len(line) - 1)
    interior = [
        (line[k + 1] - line[k - 1]) / (2.0 * delta_t) for k in range(1, len(line) - 1)
    ]
    return [np.zeros_like(line[0])] + interior + [np.zeros_like(line[0])]


def path_to_states(path, state_count, total_time, span):
    """Return support states and central-difference velocities (zero at both ends)."""
    line = resample_path(path, state_count, span)
    return line, central_velocities(line, total_time)

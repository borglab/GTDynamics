#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------

## @file sequence.py
# @brief Maneuver sequence for placing and nailing each member of a post-and-lintel build.
# @author GTDynamics contributors

"""Ordered maneuvers (segments) that place one member, after the paper's ``segments.py``.

Arm 1 (gripper) reaches the staged member, grips it, lifts, carries it into place
and holds it while arm 2 (nailer) drives nails; then arm 1 releases and clears.
Each segment lists a goal per arm (a tool pose to solve by inverse kinematics, a
joint vector, or ``None`` to hold) and the planning context that segment needs.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import gtsam
import numpy as np

import cell
import structure as st

NAIL_STANDOFF = 0.10
NAIL_EMBED = 0.005
NAIL_OFFSETS = (-0.03, -0.01, 0.01, 0.03)
# Posts: arm 2 waits back along its approach line, clear of the gripper above the nail.
POST_WAIT_BACKOFF = 0.30
# Lintel nails sit near the top of the nailer's comfortable reach.
LINTEL_HOVER_HEIGHT = 0.15
CARRY_APPROACH_HEIGHT = 0.35
CLEAR_HEIGHT = 0.40
READY_HEIGHT = 1.8


@dataclass
class Blend:
    """A joint-space waypoint partway toward a pose, shared by the segments of one block.

    The driver solves the pose once per ``block`` and sets each segment's goal a fraction of the
    way from the block's start configuration, in proportion to the other arm's tool travel.
    """

    block: str
    end: gtsam.Pose3 | np.ndarray


@dataclass
class Segment:
    """One planned maneuver: per-arm goals plus its collision and tool context."""

    key: str
    label: str
    member: str
    goals: tuple
    straight: tuple[bool, bool] = (False, False)
    carried: bool = False
    carried_other_only: bool = False
    block_clearance: float | None = None
    use_rrt: bool = True
    contact: bool = False
    nailing: bool = False
    rrt_via: tuple | None = None
    event: str | None = None
    event_at: str = "end"
    nails: list = field(default_factory=list)


def lintel_nails(member: st.Member, posts: list[st.Member]) -> list[gtsam.Pose3]:
    """Return straight-down nail goals through a lintel into each post top (four per post)."""
    top = member.target[2, 3] + member.size[2] / 2.0 - NAIL_EMBED
    width_axis = member.target[:3, 1]
    goals = []
    for post in posts:
        center = post.target[:3, 3]
        for offset in NAIL_OFFSETS:
            point = center[:2] + offset * width_axis[:2]
            goals.append(cell.downward_goal(point[0], point[1], top, 0.0))
    return goals


def post_nails(member: st.Member, structure: st.Structure) -> list[gtsam.Pose3]:
    """Return horizontal side-nail goals through a post into the base plank end, if there is a base."""
    base = next((m for m in structure.members if m.kind == "base"), None)
    if base is None:
        return []
    center = member.target[:3, 3]
    inward = 1.0 if center[1] < structure.site[1] else -1.0
    nail_y = center[1] - inward * member.size[2] / 2.0
    z = base.target[2, 3]
    return [
        cell.approach_goal(center[0] + dx, nail_y, z, (0.0, inward, 0.0))
        for dx in NAIL_OFFSETS
    ]


def member_nails(member: st.Member, structure: st.Structure) -> list[gtsam.Pose3]:
    """Return the nail goals for a member."""
    if member.kind == "lintel":
        index = int(member.name.split("_")[1]) - 1
        between = structure.spec["lintels"][index]["between"]
        posts = [
            m for m in structure.members if m.name in {f"post_{i + 1}" for i in between}
        ]
        return lintel_nails(member, posts)
    if member.kind == "post":
        return post_nails(member, structure)
    return []


def member_segments(
    member: st.Member, structure: st.Structure, previous: st.Member | None, last: bool
) -> list[Segment]:
    """Return the ordered segments that place and fasten one movable member.

    Both arms move in every maneuver except while arm 1 must hold the plank still for nailing.
    Arm 2 parks while arm 1 clears, reaches and grips (``Blend`` goals), travels from park to its
    first nail hover during the carry, and withdraws while arm 1 backs off after the release. For
    the first member arm 2 starts parked, so it travels to the hover across the whole fetch. A lintel
    comes down through arm 2's nail site, so arm 2 stays parked for its carry and sweeps to the first
    nail afterwards. Arm 1 holds until every nail is in. The build ends on the last member's last nail,
    where both arms freeze.
    """
    name = member.name
    parks = tuple(cell.park_configuration(arm) for arm in (1, 2))
    grip, hold = member.grip_staging, member.grip
    reach = st.standoff(grip)
    nails = member_nails(member, structure)
    standoffs = [st.standoff(nail, NAIL_STANDOFF) for nail in nails]
    ready = cell.downward_goal(
        structure.site[0] + 1.4, structure.site[1], READY_HEIGHT, 0.0
    )

    def waiting(standoff: gtsam.Pose3) -> gtsam.Pose3:
        return (
            st.lifted(standoff, LINTEL_HOVER_HEIGHT)
            if member.kind == "lintel"
            else st.standoff(standoff, POST_WAIT_BACKOFF)
        )

    first = waiting(standoffs[0]) if standoffs else ready
    fetch_goal = (
        Blend(f"{name}:fetch", first)
        if previous is None
        else Blend(f"{name}:park", parks[1])
    )
    lintel = member.kind == "lintel"
    carry_goal = None if lintel else (fetch_goal if previous is None else first)
    segments = []
    if previous is not None:
        # Lift straight out of the previous release first, so the long transit starts from a free pose.
        segments.append(
            Segment(
                f"{name}:clear",
                f"{name}: clear the placed member",
                name,
                (st.lifted(st.standoff(previous.grip), CLEAR_HEIGHT), fetch_goal),
                straight=(True, False),
                block_clearance=0.0,
                contact=True,
            )
        )
    segments += [
        Segment(
            f"{name}:reach",
            f"{name}: reach to staged member",
            name,
            (reach, fetch_goal),
        ),
        Segment(
            f"{name}:grip",
            f"{name}: grip",
            name,
            (grip, fetch_goal),
            straight=(True, False),
            block_clearance=0.0,
            contact=True,
            event="grip_close",
        ),
        Segment(
            f"{name}:carry",
            f"{name}: carry into place",
            name,
            (hold, carry_goal),
            carried=True,
            block_clearance=0.0,
            rrt_via=(reach, st.lifted(hold, CARRY_APPROACH_HEIGHT)),
        ),
    ]
    for i, nail in enumerate(nails):
        # A lintel's first nail on each post top is a long move rather than a nailing step.
        sweep = lintel and i % len(NAIL_OFFSETS) == 0
        segments.append(
            Segment(
                f"{name}:reposition:{i}",
                f"{name}: reposition nailer",
                name,
                (None, standoffs[i]),
                carried=True,
                carried_other_only=True,
                block_clearance=0.0,
                use_rrt=sweep,
                contact=True,
                nailing=not sweep,
            )
        )
        segments.append(
            Segment(
                f"{name}:nail:{i}",
                f"{name}: nail",
                name,
                (None, nail),
                straight=(False, True),
                carried=True,
                carried_other_only=True,
                block_clearance=0.0,
                use_rrt=False,
                contact=True,
                nailing=True,
                event="fire_nail",
            )
        )
        if last and i == len(nails) - 1:
            return segments
        segments.append(
            Segment(
                f"{name}:withdraw:{i}",
                f"{name}: withdraw",
                name,
                (None, standoffs[i]),
                straight=(False, True),
                carried=True,
                carried_other_only=True,
                block_clearance=0.0,
                use_rrt=False,
                contact=True,
                nailing=True,
            )
        )
    hover = waiting(standoffs[-1]) if standoffs else ready
    segments.append(
        Segment(
            f"{name}:release",
            f"{name}: open gripper, back off and lift the nailer",
            name,
            (st.standoff(hold), hover),
            straight=(True, False),
            block_clearance=0.0,
            contact=True,
            event="grip_open",
            event_at="start",
        )
    )
    if last:
        exit_block = f"{name}:exit"
        to_park = Blend(exit_block, parks[1])
        segments += [
            Segment(
                f"{name}:clear",
                f"{name}: clear the placed member",
                name,
                (st.lifted(st.standoff(hold), CLEAR_HEIGHT), to_park),
                straight=(True, False),
                block_clearance=0.0,
                contact=True,
            ),
            Segment(
                f"{name}:finish",
                "return both arms to park",
                name,
                (parks[0], to_park),
                block_clearance=0.0,
            ),
        ]
    return segments


def build_sequence(structure: st.Structure) -> list[Segment]:
    """Return every segment for the build, in member order."""
    movable = [member for member in structure.members if not member.fixed]
    segments: list[Segment] = []
    for index, member in enumerate(movable):
        previous = movable[index - 1] if index else None
        segments += member_segments(
            member, structure, previous, last=index == len(movable) - 1
        )
    return segments

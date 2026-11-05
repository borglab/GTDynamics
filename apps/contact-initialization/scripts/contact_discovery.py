# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file contact_discovery.py
# @brief Refine measured contact handovers.
# @author GTDynamics contributors
"""Refine contact handovers using measured sole height and speed evidence."""


def contact_component_count(contacts: list[dict]) -> int:
    """Count connected stance intervals by inclusive overlap, before any sampling."""
    components = 0
    last = -1
    for contact in sorted(contacts, key=lambda contact: contact["start"]):
        if contact["start"] > last:
            components += 1
        last = max(last, contact["end"])
    return components


def refine_handovers(support, clearances, speeds, on_height, off_speed):
    """Look back one sample from a confirmed direct left/right handover.

    Acquisition hysteresis can delay a touchdown until just after the other
    foot's final supported sample. Recover that shared boundary only when the
    incoming foot already meets the height-on and speed-hold thresholds. Never
    extend across a sample where neither foot was supported.
    """
    refined = [list(row) for row in support]
    corrections = []
    for i in range(1, len(support)):
        for incoming in range(2):
            outgoing = 1 - incoming
            if (
                support[i - 1][outgoing]
                and not support[i - 1][incoming]
                and support[i][incoming]
                and not support[i][outgoing]
                and clearances[i - 1][incoming] <= on_height
                and speeds[i - 1][incoming] <= off_speed
            ):
                refined[i - 1][incoming] = True
                corrections.append(
                    {
                        "sampleIndex": i - 1,
                        "confirmedOnsetSample": i,
                        "foot": ("left", "right")[incoming],
                    }
                )
    return refined, corrections

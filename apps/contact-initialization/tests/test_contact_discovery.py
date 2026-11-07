# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file test_contact_discovery.py
# @brief Verify contact handover refinement.
# @author GTDynamics contributors
"""Contact handovers must share a frame without bridging genuine flight."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from contact_discovery import contact_component_count, refine_handovers


class TestContactDiscovery(unittest.TestCase):
    def test_contact_components_require_shared_samples(self):
        contacts = [dict(start=0, end=4), dict(start=4, end=8), dict(start=9, end=12)]
        self.assertEqual(contact_component_count(contacts), 2)
        self.assertEqual(contact_component_count([]), 0)

    def test_confirmed_handover_recovers_shared_boundary(self):
        support = [[True, False], [True, False], [False, True], [False, True]]
        heights = [[0.0, 0.1], [0.0, 0.033], [0.01, 0.03], [0.02, 0.01]]
        speeds = [[0.0, 2.0], [0.8, 0.51], [1.75, 0.27], [2.0, 0.1]]
        refined, corrections = refine_handovers(support, heights, speeds, 0.04, 1.0)
        self.assertEqual(refined[1], [True, True])
        self.assertEqual(
            corrections,
            [{"sampleIndex": 1, "confirmedOnsetSample": 2, "foot": "right"}],
        )
        self.assertEqual(support[1], [True, False])

    def test_flight_gap_remains_unsupported(self):
        support = [[True, False], [False, False], [False, False], [False, True]]
        refined, corrections = refine_handovers(
            support, [[0.0, 0.0]] * 4, [[0.0, 0.0]] * 4, 0.04, 1.0
        )
        self.assertEqual(refined, support)
        self.assertEqual(corrections, [])

    def test_height_and_speed_evidence_are_required(self):
        support = [[True, False], [False, True]]
        for height, speed in [(0.06, 0.1), (0.03, 1.5)]:
            refined, corrections = refine_handovers(
                support,
                [[0.0, height], [0.0, 0.0]],
                [[0.0, speed], [0.0, 0.0]],
                0.04,
                1.0,
            )
            self.assertEqual(refined, support)
            self.assertEqual(corrections, [])


if __name__ == "__main__":
    unittest.main()

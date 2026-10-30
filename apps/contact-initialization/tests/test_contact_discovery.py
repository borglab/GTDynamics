"""Contact handovers must share a frame without bridging genuine flight."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from contact_discovery import component_count, refine_handovers, sample_frames


class TestContactDiscovery(unittest.TestCase):
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


class TestFrameSampling(unittest.TestCase):
    def test_long_stance_uses_original_samples_at_one_second_intervals(self):
        samples = [{"t": round(i / 60, 8)} for i in range(194)]
        contacts = [{"id": "L00", "start": 0, "end": 193, "endExclusive": 194}]
        frames, sampling = sample_frames(samples, contacts)
        self.assertEqual([f["sampleIndex"] for f in frames], [0, 60, 120, 180, 193])
        self.assertEqual(
            [f["selection"] for f in frames],
            ["contact", "periodic", "periodic", "periodic", "contact"],
        )
        self.assertTrue(all(f["contacts"] == ["L00"] for f in frames))
        self.assertEqual(sampling["periodicFrameCount"], 3)
        self.assertEqual(component_count(frames), 1)

    def test_frequent_contact_boundaries_need_no_periodic_frames(self):
        samples = [{"t": i / 4} for i in range(9)]
        contacts = [
            {"id": "L00", "start": 0, "end": 4, "endExclusive": 5},
            {"id": "R00", "start": 3, "end": 8, "endExclusive": 9},
        ]
        frames, sampling = sample_frames(samples, contacts)
        self.assertEqual([f["sampleIndex"] for f in frames], [0, 3, 4, 8])
        self.assertEqual(sampling["periodicFrameCount"], 0)
        self.assertEqual(component_count(frames), 1)

    def test_periodic_frames_restart_after_each_contact_boundary(self):
        samples = [{"t": i / 10} for i in range(35)]
        contacts = [
            {"id": "L00", "start": 0, "end": 14, "endExclusive": 15},
            {"id": "R00", "start": 13, "end": 34, "endExclusive": 35},
        ]
        frames, _ = sample_frames(samples, contacts)
        self.assertEqual([f["sampleIndex"] for f in frames], [0, 10, 13, 14, 24, 34])
        self.assertEqual(frames[2]["contacts"], ["L00", "R00"])
        self.assertEqual(frames[4]["contacts"], ["R00"])

    def test_sampling_does_not_extend_contact_across_flight(self):
        samples = [{"t": i / 10} for i in range(41)]
        contacts = [
            {"id": "L00", "start": 0, "end": 10, "endExclusive": 11},
            {"id": "R00", "start": 30, "end": 40, "endExclusive": 41},
        ]
        frames, _ = sample_frames(samples, contacts)
        self.assertEqual([f["sampleIndex"] for f in frames], [0, 10, 20, 30, 40])
        self.assertEqual(frames[2]["contacts"], [])

    def test_sparse_or_invalid_source_timestamps_are_rejected(self):
        contacts = [{"id": "L00", "start": 0, "end": 1, "endExclusive": 2}]
        for times in ([0, 1.5], [0, 0], [0, float("nan")]):
            with self.assertRaises(ValueError):
                sample_frames([{"t": t} for t in times], contacts)


if __name__ == "__main__":
    unittest.main()

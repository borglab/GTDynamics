"""Integration check: reference geometry cannot enter native pose initialization."""

from copy import deepcopy
import importlib.util
import json
import gzip
import gtdynamics as gtd
from pathlib import Path
import unittest

import numpy as np

APP = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "export_pose_recordings", APP / "scripts/export_native_recordings.py"
)
exporter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(exporter)


class PoseRecordingTests(unittest.TestCase):
    def test_reference_is_held_out(self) -> None:
        original = json.loads((APP / "public/g1-walk.json").read_text())
        changed = deepcopy(original)
        for reference in changed["reference"]["poses"]:
            reference["p"] = [100.0, 200.0, -300.0]
            reference["q"] = [0.5, 0.5, 0.5, 0.5]
        model = exporter.load_g1_model()
        results = []
        for data in (original, changed):
            robot, measured, trajectory, anchor = exporter.prepare_inputs(data, model)
            result, _ = exporter.record_run(
                robot, measured, trajectory, anchor, 0.01, False, True, 2, 1, 0
            )
            self.assertAlmostEqual(
                result.graph.error(result.values) * 2, result.objectiveFinal
            )
            results.append(result)
        self.assertEqual(
            [s.k for s in results[0].slices()], [s.k for s in results[1].slices()]
        )
        for a, b in zip(results[0].poses, results[1].poses):
            np.testing.assert_allclose(a.matrix(), b.matrix(), atol=1e-12)
        for a, b in zip(results[0].contactPoses, results[1].contactPoses):
            np.testing.assert_allclose(a.matrix(), b.matrix(), atol=1e-12)

    def test_rejected_steps_are_stalled_not_converged(self) -> None:
        """Reproduce the middle snippet's wrong-ground stop with nonzero gradient."""
        data = json.loads(
            gzip.decompress((APP / "public/snippets/ssio-middle.json.gz").read_bytes())
        )
        robot, measured, trajectory, body_t_com = exporter.prepare_inputs(
            data, exporter.load_g1_model()
        )
        options = gtd.pose_contact_init.Options()
        options.selectedSlices = gtd.doubleSupportSlices(trajectory, 0.02, 0.2, 1)
        options.interpolateOutput = False
        options.rotationSigma = float(np.deg2rad(90.0))
        options.addUprightFactors = True
        options.addNormalFactors = False
        options.groundSigma = 0.1
        options.groundHeight = 0.0
        first = options.selectedSlices[0].k
        reference = data["reference"]["poses"][first]
        # Deliberately undo the frame correction to reproduce the original bug.
        position = (
            np.array(reference["p"]) - data["metadata"]["snippetWorldTranslation"]
        )
        anchor = exporter.pose(position, reference["q"]) * body_t_com
        result, _ = exporter.record_run(
            robot,
            measured,
            trajectory,
            body_t_com,
            0.02,
            True,
            True,
            200,
            1,
            0,
            options,
            anchor,
        )
        self.assertTrue(result.stalled)
        self.assertFalse(result.converged)
        self.assertAlmostEqual(
            result.history[-1].objective, result.history[-2].objective
        )
        matrix, residual = result.graph.linearize(result.values).jacobian()
        self.assertGreater(np.linalg.norm(matrix.T @ residual), 1e-3)


if __name__ == "__main__":
    unittest.main()

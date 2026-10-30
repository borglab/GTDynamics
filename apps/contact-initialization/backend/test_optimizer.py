"""Native CLI integration: real G1 geometry, no reference use, honest residuals."""

import copy
import json
from itertools import product
from pathlib import Path
import subprocess
import sys

binary = sys.argv[1]
source = Path(__file__).resolve().parents[1] / "public" / "g1-walk.json"
dataset = json.loads(source.read_text())
reference = dataset.pop("reference")
dataset.pop("metadata", None)


def solve(data, model, regularization=None, evaluation_reference=None):
    request = {
        "dataset": data,
        "model": model,
        "iterations": 100,
        "repetitions": 1,
        "warmup": 0,
    }
    if regularization is not None:
        request["regularization"] = regularization
    if evaluation_reference is not None:
        request["evaluationReference"] = evaluation_reference
    completed = subprocess.run(
        [binary], input=json.dumps(request), text=True, capture_output=True, check=True
    )
    return json.loads(completed.stdout)


for model in ("ground", "patch", "point"):
    native = solve(dataset, model)
    graph, result = native["graph"], native["result"]
    assert len(graph["frames"]) == len(result["poses"]) == 75
    assert len(graph["contacts"]) == 38
    assert graph["observability"]["componentCount"] == 1
    assert graph["dimensions"]["jointVariables"] == 0
    assert (
        graph["dimensions"]["optimizedScalars"]
        == {"ground": 672, "patch": 786, "point": 558}[model]
    )
    assert result["poses"][0] == graph["seedPoses"][0]
    assert result["objectiveFinal"] <= result["objectiveInitial"]
    assert result["solver"] == "gtsam::LevenbergMarquardtOptimizer"
    assert len(result["history"]) == result["iterations"] + 1
    for before, after in zip(result["history"], result["history"][1:]):
        assert after["objective"] <= before["objective"] + 1e-12
    # Independently recompute every contact residual. There are no hidden pose,
    # motion, gravity or reference terms in the reported native objective.
    objective = 0.0
    observations = 0
    for frame, pose in zip(graph["frames"], result["poses"]):
        rotation, translation = pose["rotation"], pose["translation"]
        for observation in frame["observations"]:
            landmark = result["landmarks"][observation["landmarkIndex"]]
            local = observation["local"]
            for row in range(3):
                predicted = translation[row] + sum(
                    rotation[3 * row + col] * local[col] for col in range(3)
                )
                objective += (predicted - landmark[row]) ** 2
            observations += 1
    assert abs(objective - result["objectiveFinal"]) < 1e-10
    assert abs((objective / observations) ** 0.5 - result["residualRms"]) < 1e-12
    if model == "ground":
        height = graph["groundPlane"]["height"]
        assert all(point[2] == height for point in result["landmarks"])
    for phase in ("graphBuild", "seed", "optimization", "total"):
        statistics = native["benchmark"]["phases"][phase]
        assert len(statistics["samplesMs"]) == 1
        assert statistics["medianMs"] >= 0
    poisoned = copy.deepcopy(dataset)
    poisoned["reference"] = "Reference data must never be parsed by the solver."
    alternate = solve(poisoned, model)
    assert alternate["result"]["poses"] == result["poses"]
    assert alternate["result"]["landmarks"] == result["landmarks"]
    for between, upright, ground in product((False, True), repeat=3):
        if not (between or upright or ground):
            continue
        regularization = {"between": between, "upright": upright, "ground": ground}
        saved = solve(dataset, model, regularization, reference)
        components = saved["result"]["objectiveComponents"]
        assert abs(sum(components.values()) - saved["result"]["objectiveFinal"]) < 1e-9
        assert len(saved["graph"]["betweenEdges"]) == (74 if between else 0)
        assert len(saved["graph"]["uprightPoseIndices"]) == (75 if upright else 0)
        count = len(saved["graph"]["landmarks"]) if ground and model != "ground" else 0
        assert len(saved["graph"]["groundLandmarkIndices"]) == count
        assert saved["graph"]["dimensions"]["groundFactors"] == count
        if model == "patch" and ground and not upright:
            # A rocking-foot seed previously trapped these solves upside down
            # at more than 1.6 m RMSE, even with the height prior enabled.
            assert saved["evaluation"]["positionRmseMeters"] < 0.35
            assert saved["evaluation"]["orientationRmseDegrees"] < 20
        if count:
            settings = saved["graph"]["regularization"]
            cost = sum(
                ((point[2] - settings["groundHeight"]) / settings["groundSigma"]) ** 2
                for point in saved["result"]["landmarks"]
            )
            assert abs(cost - components["ground"]) < 1e-9
            assert saved["graph"]["groundPlane"] is None
        else:
            assert components["ground"] == 0
        corrupted = copy.deepcopy(reference)
        for i, pose in enumerate(corrupted["poses"]):
            pose["p"][0] += 1000 * i
        checked = solve(dataset, model, regularization, corrupted)
        assert checked["result"]["poses"] == saved["result"]["poses"]
        assert checked["result"]["landmarks"] == saved["result"]["landmarks"]
        assert (
            checked["evaluation"]["positionRmseMeters"]
            > saved["evaluation"]["positionRmseMeters"]
        )

print(
    "Native G1 models, objective, hard floor, dimensions, timings and reference isolation passed."
)

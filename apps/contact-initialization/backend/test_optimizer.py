"""Native CLI integration: real G1 geometry, no reference use, honest residuals."""
import copy
import json
from pathlib import Path
import subprocess
import sys

binary = sys.argv[1]
source = Path(__file__).resolve().parents[1] / "public" / "g1-walk.json"
dataset = json.loads(source.read_text())
dataset.pop("reference", None)
dataset.pop("metadata", None)


def solve(data, model):
    request = {"dataset": data, "model": model, "iterations": 100,
               "repetitions": 1, "warmup": 0}
    completed = subprocess.run([binary], input=json.dumps(request), text=True,
                               capture_output=True, check=True)
    return json.loads(completed.stdout)


for model in ("ground", "patch", "point"):
    native = solve(dataset, model)
    graph, result = native["graph"], native["result"]
    assert len(graph["frames"]) == len(result["poses"]) == 75
    assert len(graph["contacts"]) == 38
    assert graph["observability"]["componentCount"] == 1
    assert graph["dimensions"]["jointVariables"] == 0
    assert graph["dimensions"]["optimizedScalars"] == {
        "ground": 672, "patch": 786, "point": 558}[model]
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
                    rotation[3 * row + col] * local[col] for col in range(3))
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

print("Native G1 models, objective, hard floor, dimensions, timings and reference isolation passed.")

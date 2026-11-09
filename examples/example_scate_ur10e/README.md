# SCATE on the UR10e

The SCATE example of [`example_scate_ur5`](../example_scate_ur5/README.md) on
the UR10e model in `models/urdfs/ur10e`: the same offline plan, online loop and
scenario (start, goals, obstacle placement, sensor range, noise, planner
settings), with the UR10e's collision spheres and a larger obstacle SDF grid.

| File | What it holds |
| :--- | :--- |
| `ur10e_collision.h` | UR10e robot file, spheres, self-collision pairs and SDF grid (`Ur10eCollisionModel`) |
| `scate_ur10e_init.cpp` | Offline plan (`runOfflineExample` in `scate_scenario.h`) |
| `scate_ur10e_online.cpp` | Closed loop and blind control (`runOnlineExample` in `scate_scenario.h`) |

Everything else (graph, online planner, simulation, replay writer, the two
program bodies) is the header-only code in `example_scate_ur5`, which this
example adds to its include path. The UR10e URDF uses the ROS 2 description's
link frames (arm links along -x, the wrist_3 frame at the tool flange), but the
same joint-angle convention as the UR5, so the same joint vectors give the same
posture on a larger arm (reach about 1.3 m instead of 0.85 m).

## Running

From the GTDynamics build folder:

```bash
make -j6 scate_ur10e_init.run      # offline plan, writes initial/optimized_trajectory.csv
make -j6 scate_ur10e_online.run    # closed loop and blind control, writes replay.json
make scate_ur10e_online.plot       # PNG plots and scate_replay.html
make scate_ur10e_online.export     # copy replay.json into the SCATE UR10e app
```

Outputs land in `build/examples/example_scate_ur10e`. Both programs take
another robot file as the first argument; `scate_ur10e_online` takes a random
seed as the second.

The 3D viewer is the [SCATE UR10e app](../../apps/scate/ur10e/README.md).

The UR10e tests are `testUr10eScateInit` (loading, FK against an independent
numpy FK, IK for the scenario goals, the offline graph) and `testUr10eCollision`
(spheres, sensor, link frames, SDF grid, factor Jacobians):
`make -j6 testUr10eScateInit.run testUr10eCollision.run`. The planner itself is
tested robot-independently in `testScateOnline` and `testScateGraph`.

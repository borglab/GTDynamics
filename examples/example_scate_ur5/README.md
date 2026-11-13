# SCATE on the UR5

Joint-space SCATE (simultaneous control and trajectory estimation) for a UR5
arm, after Bhattacharyya et al. A factor graph over joint angles, velocities
and accelerations is planned offline, then re-solved after every executed step
as encoder readings and obstacle measurements arrive (the paper's Algorithm 1).

| File | What it holds |
| :--- | :--- |
| `scate_keys.h` | Variable keys `q_i`, `v_i`, `u_i` |
| `trajectory_init.h` | Smooth rest-to-rest initial guess, double-integrator model |
| `build_scate_graph.h` | Offline graph: endpoint priors, `DoubleIntegratorFactor`, `JointLimitFactorVector` hinges |
| `scate_init.h` | UR5 loading, FK/IK, start and goal, robot limits to `GraphSpec` |
| `scate_online.h` | `CollisionModel` interface, `ObstacleMeasurementFactor`, `OnlinePlanner`, closed-loop simulation |
| `sphere_collision.h` | `SphereCollisionModel`: any arm's collision spheres with `ObstacleSDFFactorGP` and `SelfCollisionSphereFactorGP` |
| `ur5_collision.h` | The UR5's spheres and self-collision pairs (`Ur5CollisionModel`) |
| `scate_scenario.h` | The bodies of `scate_init` and `scate_online`, shared with the UR10e |
| `scate_reactive.h` | Reactive SCATE with motion-capture sensing and moving obstacles: `MarkerMeasurementFactor`, `ReactivePlanner`, closed loop |
| `scate_reactive_scenario.h` | The body of `scate_ur10e_reactive`: obstacles crossing the plan, reactive vs frozen vs blind |
| `base_wrench.h` | Motion-induced wrench on the base (gravity-free Newton-Euler) and `BaseWrenchFactor`, the base-disturbance cost |
| `scate_replay.h` | Writes a run to `replay.json` |

## Running

From the GTDynamics build folder:

```bash
make -j6 scate_init.run      # offline plan, writes initial/optimized_trajectory.csv
make -j6 scate_online.run    # closed loop and blind control, writes replay.json
make scate_online.plot       # PNG plots and scate_replay.html
make scate_online.export     # copy replay.json into the SCATE app
```

Outputs land in `build/examples/example_scate_ur5`. Both programs load
`models/urdfs/ur5/ur5.urdf` unless given another robot file as the first
argument; `scate_online` takes a random seed as the second.

The 3D viewer and the plot script live in the SCATE app,
[`apps/scate/ur5`](../../apps/scate/ur5/README.md), which replays the
checked-in run in the GTDynamics apps site.

The same scenario runs on the UR10e in
[`examples/example_scate_ur10e`](../example_scate_ur10e/README.md), which
reuses every header here except `ur5_collision.h`.

The tests are in `tests/`: `testScateReactive` (reactive SCATE on a toy
robot), `testBaseWrench` (base wrench against GTDynamics' inverse dynamics
and momentum conservation), `testTrajectoryInit`, `testScateGraph`,
`testScateInit`, `testUr5Collision` and `testScateOnline`
(`make -j6 testScateOnline.run`). The dynamics factor has its own test,
`testDoubleIntegratorFactor`.

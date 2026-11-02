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
| `ur5_collision.h` | UR5 collision spheres with `ObstacleSDFFactorGP` and `SelfCollisionSphereFactorGP` |
| `scate_replay.h` | Writes a run to `replay.json` |
| `plot_scate.py`, `viz/` | Plots and a 3D replay page from `replay.json` (numpy, matplotlib) |

## Running

From the GTDynamics build folder:

```bash
make -j6 scate_init.run      # offline plan, writes initial/optimized_trajectory.csv
make -j6 scate_online.run    # closed loop and blind control, writes replay.json
make scate_online.plot       # PNG plots and scate_replay.html
```

Outputs land in `build/examples/example_scate_ur5`. Both programs load
`models/urdfs/ur5/ur5.urdf` unless given another robot file as the first
argument; `scate_online` takes a random seed as the second.

The tests are in `tests/`: `testTrajectoryInit`, `testScateGraph`,
`testScateInit`, `testUr5Collision` and `testScateOnline`
(`make -j6 testScateOnline.run`). The dynamics factor has its own test,
`testDoubleIntegratorFactor`.

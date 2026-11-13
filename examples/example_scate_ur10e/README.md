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
| `scate_ur10e_reactive.cpp` | Reactive SCATE with motion capture and two moving obstacles (`runReactiveExample` in `scate_reactive_scenario.h`) |
| `scate_ur10e_base_wrench.cpp` | The base-disturbance cost off and at several sigmas, with and without obstacles |
| `ur10e_scenarios.h` | The two moving obstacles shared by both programs |

Everything else (graph, online planner, simulation, replay writer, the two
program bodies) is the header-only code in `example_scate_ur5`, which this
example adds to its include path. The UR10e URDF uses the ROS 2 description's
link frames (arm links along -x, the wrist_3 frame at the tool flange), but the
same joint-angle convention as the UR5, so the same joint vectors give the same
posture on a larger arm (reach about 1.3 m instead of 0.85 m).

## Minimizing the disturbance on the base

In orbit the arm sits on a spacecraft whose attitude control holds the base
still, and every wrench the arm puts on its mount is a disturbance to cancel.
With no gravity that wrench is the motion-induced part of the inverse
dynamics, which `base_wrench.h` computes (Newton-Euler, gravity off) and
`BaseWrenchFactor` pushes toward zero on the intervals still ahead. The
weight of the arm, which only exists on Earth, is left out.

```bash
./scate_ur10e_reactive ../../../models/urdfs/ur10e/ur10e.urdf 0 base   # reactive run with the cost
make scate_ur10e_base_wrench.run    # the study below, 3 seeds (pass a count to the program for more)
```

Over seeds 0-9 with the two moving obstacles, the default sigma (3 N m, 3 N)
cut the base torque from 1.06 to 0.18 N m RMS, the peak from 2.9 to 0.43 N m
and its time integral from 16.0 to 3.0 N m s, with no collisions and slightly
more clearance (worst 3.3 cm instead of 3.0 cm). Smaller sigmas reduce the
wrench a little more but give up clearance (0.3: worst 0.8 cm; 0.1: 5 of 10
runs collide). On the sweep alone the cost removes about 12-20%, since that
motion is already gentle.

The study also writes `base_wrench_replay.json`: seed 0 with the cost off and
at sigma 3, 0.3 and 0.1, with the base wrench in every frame. It is the
recording of the SCATE UR10e Base app (`apps/scate/ur10e-reactive`), where
the runs can be toggled and compared:

```bash
make scate_ur10e_base_wrench.run      # writes base_wrench_replay.json
make scate_ur10e_base_wrench.export   # copies it to apps/scate/ur10e-reactive/public/replay.json
```

## Running

From the GTDynamics build folder:

```bash
make -j6 scate_ur10e_init.run      # offline plan, writes initial/optimized_trajectory.csv
make -j6 scate_ur10e_online.run    # closed loop and blind control, writes replay.json
make scate_ur10e_online.plot       # PNG plots and scate_replay.html
make scate_ur10e_online.export     # copy replay.json into the SCATE UR10e app
```

Outputs land in `build/examples/example_scate_ur10e`.

## Reactive SCATE with motion capture

`scate_ur10e_reactive` replaces the wrist range sensor with a VICON-style
system that measures, in the world frame, 8 markers on the arm and the centers
of two moving obstacles every second. The planner follows the paper's reactive
SCATE (eq. 7, Algorithm 1): one location variable per obstacle per step, and
after every reading all obstacle factors on the remaining intervals are rebuilt
from the latest estimates, as if the obstacles stayed where they were last
seen. Obstacle 0 moves toward the base and crosses the tool's path at t = 12 s;
obstacle 1 descends onto the forearm at t = 7 s. "Frozen" (obstacles seen at
the start only) and "blind" runs on the same noise are the negative controls.

```bash
make -j6 scate_ur10e_reactive.run     # markers only; writes reactive_replay.json
./scate_ur10e_reactive ../../../models/urdfs/ur10e/ur10e.urdf 3 encoders   # seed 3, encoders too
make scate_ur10e_reactive.plot        # PNGs and scate_replay.html in reactive_plots/
```

Over seeds 0-9 the reactive run never collides (closest 3.0 cm with markers
only, 3.2 cm with encoders too), while the blind run hits an obstacle in all
20 runs and the frozen run in 19. Clearance dips below the 5 cm standoff by
about the obstacle speed times dt, because reactive planning does not predict
the motion. With markers only, the final joint error is 0.016-0.050 rad,
mostly in wrist 3, whose velocity the markers observe least; with encoders it
is 0.009-0.025 rad. Both programs take
another robot file as the first argument; `scate_ur10e_online` takes a random
seed as the second.

The 3D viewer is the [SCATE UR10e app](../../apps/scate/ur10e/README.md).

The UR10e tests are `testUr10eScateInit` (loading, FK against an independent
numpy FK, IK for the scenario goals, the offline graph) and `testUr10eCollision`
(spheres, sensor, link frames, markers, SDF grid, factor Jacobians):
`make -j6 testUr10eScateInit.run testUr10eCollision.run`. The planner itself is
tested robot-independently in `testScateOnline` and `testScateGraph`.

## Live solver for the app

`scate_live` (`live_scenario.h`) is the native side of the SCATE UR10e Base
app's live simulation, for the UR10e or the UR5 (with `ur5Markers()`):

```bash
./scate_live describe ur10e robot.json     # link frames, screw axes, spheres, default experiment
./scate_live solve request.txt replay.json # one reactive run per sigma, as one replay
```

A request is plain text: `robot ur10e`, `q_start ...`, `q_goal ...`, one
`obstacle x y z vx vy vz radius` per obstacle, `sigmas 0 3 0.3 0.1` and
`seed 0`. The goal is used as given (no IK). `scate_live.describe` rewrites
the app's `public/robots/*.json`; `testScateLive` checks the request reader,
that the screw axes reproduce the link frames, the UR5 markers, and a solve.


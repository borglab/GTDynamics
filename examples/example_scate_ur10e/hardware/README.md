# SCATE hardware runs on the UR10e

Reactive SCATE (Phase 6) on the real arm in MK 111: the planner re-plans at
every state while the arm moves, and an executor streams each new plan to the
arm as joint velocity commands, through `ur_bridge` from
[UR10e-Trajectory-Validation](https://github.com/HarrisonJenkins45/UR10e-Trajectory-Validation)
(branch `revision/ros_hardware_demo`).

```
 app (browser) ──HTTP──► dev server ──HTTP──► executor (scate_hw) ──stdin/stdout──► scate_hardware (C++)
                         (liveServer.mjs)       │ 50 Hz: v_ref + kp (q_ref - q)
                                                ▼
                                     SimRobot (laptop)  or  RosRobot ──► /ur/joint_velocity_command
                                                                          ur_bridge ──RTDE──► speedj at 125 Hz
```

## The pieces

| Piece | Where | What it does |
|---|---|---|
| `scate_hardware` | `../scate_hardware.cpp`, `../hardware_session.h` | The planner process. Simulates the **ghost run** first and checks it (`assessRun`), then `start` (initial plan), `measure k t q v obstacles` (re-plan), `trace` and `finish` (replay with the hardware run beside its ghost run). |
| `scate_hw` | `scate_hw/` (plain Python, no numpy, no ROS) | The executor: state machine, control loop, plan checks, simulated arm, HTTP control API. |
| `scate_ur10e_hw` | `ros2/scate_ur10e_hw/` (ROS 2 package) | `ur_bridge` and `hardware_protocol` copied unchanged from the repo above (only the package name and the allowed command node changed), `fake_ur` (ur_bridge's topics around the simulated arm), and `scate_executor` (the same executor with a `RosRobot`). |
| Hardware panel | `apps/scate/shared/HardwarePanel.vue` | In the SCATE UR10e Base app: plan for hardware, watch the ghost run, begin, watch the run, load the result. |

## The flow

1. In the app, tick **Plan for a hardware run**. This turns on the rig (wall
   and floor), joint encoders only (no motion capture yet), and the velocity
   and acceleration limits you set. It loads the rig experiment, which sweeps
   up and away from the floor. Drag your own start, goal and obstacles if you
   like.
2. **Solve and run**: the runs it plays are the ghost runs. Each one is checked:
   no collision, at least 1 cm from the walls and floor, joints inside their
   limits, and speed and acceleration within 105% of the limits.
3. Pick the run to execute and press **Begin hardware experiment**. The
   executor starts `scate_hardware`, which simulates the same run again (it
   is deterministic: the same request and noise seed give the same run). The
   executor refuses unless that ghost run is safe and the start and goal
   poses are clear of the wall and floor. If the arm is not at the start
   pose, **Move arm to start** brings it there at 0.2 rad/s; press Begin
   again.
4. The run takes 20 s (N = 20 states, dt = 1 s). At each state time the
   executor reads the arm (joint encoders) and the obstacles and sends them to
   the planner. It keeps following the current plan until the new one
   arrives, and checks every new plan before using it. The obstacles are
   simulated: their true centers are the readings, one per state, and the
   planner never sees their future path. That is reactive SCATE.
5. After the last state the arm holds for 1 s and stops. The app loads the
   replay with the **Hardware run** (the measured motion) beside the **Ghost
   run (simulated)**.

### The command law and what stops the arm

Every 20 ms (`rate_hz` 50) the executor sends

    v = v_ref(t) + clip(kp (q_ref(t) - q), ±max_correction),   clipped to ±command_cap

with `kp` 1.5/s, `max_correction` 0.1 rad/s and `command_cap` 0.55 rad/s.
This is plan_executor's law from the repo above. Between states the plan is
exact (constant acceleration), so `q_ref` and `v_ref` come straight from it.

The arm is stopped (zero velocity, then the planner writes the replay) on any of:

- the arm's state is older than 0.2 s;
- a joint is more than `tracking_abort` (0.08 rad) from the plan;
- a new plan fails its check: joint limits, speed or acceleration over 110%
  of the planner's limits, any collision sphere through the wall or floor
  (the executor's own forward kinematics), or a start more than 0.08 rad from
  the arm;
- a re-plan takes longer than 0.8 dt;
- the planner reports an error or exits;
- the arm reports a fault (the bridge's status is not `ready`);
- **Stop arm** in the app.

The bridge has its own guards too: zero velocity if commands stop for 0.25 s,
a fault above `max_joint_speed` (0.6 rad/s, above the executor's 0.55 cap),
and the checks of robot mode, safety mode and speed scaling.

## Testing on the Mac (no ROS, no robot)

```bash
cd ~/dev/GTDynamics/build
cmake .. && ninja scate_live scate_hardware testScateHardware
./tests/testScateHardware

cd ../examples/example_scate_ur10e/hardware
python3 -m unittest discover -s tests        # about 35 s; the end-to-end runs are real time

cd ~/dev/GTDynamics/apps
npm run dev --workspace scate/ur10e-reactive # http://127.0.0.1:3012/
```

With nothing listening on 127.0.0.1:8765, the dev server starts the executor
itself with a simulated arm (`python3 -m scate_hw`). The panel then says
"Executor: simulated arm". To run it yourself instead, for example to see its
log or to start the arm away from the start pose:

```bash
cd ~/dev/GTDynamics/examples/example_scate_ur10e/hardware
python3 -m scate_hw --start-offset 0.1      # then npm run dev as above
```

The browser test of the whole flow is `apps/scate/ur10e-reactive/tests/hardware.browser.mjs`
(Playwright and Chrome; about a minute).

## In the lab (Ubuntu, ROS 2 Humble)

Build GTSAM and GTDynamics on the lab PC (GTSAM at GTDynamics' CI pin
`c940b441`), then `ninja scate_live scate_hardware` in GTDynamics' `build/`.
Then:

```bash
mkdir -p ~/scate_ws/src && ln -s ~/dev/GTDynamics/examples/example_scate_ur10e/hardware/ros2/scate_ur10e_hw ~/scate_ws/src/
cd ~/scate_ws && colcon build --symlink-install && source install/setup.bash
export GTDYNAMICS_DIR=~/dev/GTDynamics

ros2 launch scate_ur10e_hw scate_hardware.launch.py                           # 1. fake arm through ROS
ros2 launch scate_ur10e_hw scate_hardware.launch.py hardware:=real            # 2. shadow mode
ros2 launch scate_ur10e_hw scate_hardware.launch.py hardware:=real enable_commands:=true   # 3. real
```

and in another terminal `npm run dev --workspace scate/ur10e-reactive` in
`~/dev/GTDynamics/apps` (with the launch running, the dev server finds the
executor and does not start a simulated one).

- **Fake arm** (step 1) checks the ROS wiring with nothing leaving the PC.
- **Shadow mode** (step 2): the bridge reads the real encoders and logs what it
  would send. The arm does not move, so a run stops on tracking error within
  a second. Use it to check the connection and the pre-start checks (the
  start pose, the ghost run).
- **Real** (step 3): reduced safety configuration on the pendant, speed
  slider at 100% (the bridge requires it), robot in Remote Control, hand on
  the e-stop. Start with low limits (velocity limit 0.2 rad/s in the app) and
  no obstacle near the path.

Things to check on the first real runs:

- the mounting offset of the carriage (2.5 cm, a TODO in the repo's URDF):
  the wall plane is that close to the base;
- tracking error during a run (shown in the panel): if the arm lags,
  raise `speedj_acceleration` (default 2.0 rad/s²) or `kp`;
- `sigma_executed_control` in the request (default 0.01 rad/s²): the
  planner's trust in the executed controls. Tune it to the real tracking error.

## Not verified

The ROS 2 package has not been run under ROS here. The adapter's logic is
tested with stand-in ROS modules (`tests/test_ros_adapter.py`), and
`ur_bridge` is the repo's own, unchanged. Nothing has been run on the real
arm.

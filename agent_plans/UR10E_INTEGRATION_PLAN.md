# UR10e URDF Integration Plan

## Context

The goal is to add a Universal Robots **UR10e** arm model to GTDynamics, so it can be loaded with
`CreateRobotFromFile` from C++ and Python and used in kinematics, dynamics and optimization graphs.
GTDynamics does not register robots in code. A robot counts as "integrated" when its model file
sits in a known models directory. Tests (and optionally examples or demos) are what show that the
model loads and behaves correctly.

## Where existing robot models live

| Location | Contents | Pattern |
|---|---|---|
| [models/urdfs/](../models/urdfs/) | Real robots: `atlas.urdf`, `vision60.urdf`, `fanuc_lrmate200id.urdf`, `nao.urdf`, … | A robot with no meshes is a single `.urdf` file at the top level |
| [models/urdfs/ur5/](../models/urdfs/ur5/) | `ur5.urdf` + `visual/*.dae` + `collision/*.stl` | **The closest template for UR10e.** Mesh paths are relative (`visual/base.dae`) |
| [models/urdfs/panda/](../models/urdfs/panda/) | `panda.urdf` + `visual/` + `collision/` | Same subfolder pattern; also has tests ([python/tests/test_panda.py](../python/tests/test_panda.py)) |
| [models/urdfs/a1/](../models/urdfs/a1/) | `a1.urdf` + `meshes/` | Alternative mesh folder layout |
| [models/urdfs/test/](../models/urdfs/test/) | Small synthetic URDFs used only by unit tests | Not for real robots |
| [models/sdfs/](../models/sdfs/) | SDF-format robots (spider, a1, kuka world) | Only needed if you use SDF instead |
| [models/mujoco/](../models/mujoco/) | MuJoCo Menagerie models, fetched by a script (not committed) | Only used by the web demo |

Note: `ur5` is in the repo, but **no test or example loads it** (checked with grep). Panda is the
best example of a manipulator that is both stored and tested.

## Where to get the UR10e URDF

UR publishes its models as **xacro**, not as ready-made URDF, so you have to flatten them.

1. **Universal Robots ROS 2 description (recommended):**
   https://github.com/UniversalRobots/Universal_Robots_ROS2_Description (BSD-3-Clause)
   - One generic xacro covers every UR model: `urdf/ur.urdf.xacro`, selected with `ur_type:=ur10e`.
   - The UR10e-specific parameters are in `config/ur10e/`: `default_kinematics.yaml`,
     `physical_parameters.yaml` (masses and inertias), `joint_limits.yaml` and `visual_parameters.yaml`.
   - The meshes are in `meshes/ur10e/visual/*.dae` and `meshes/ur10e/collision/*.stl`.
   - Flatten it with:
     `ros2 run xacro xacro <ur_description>/urdf/ur.urdf.xacro ur_type:=ur10e name:=ur10e > ur10e.urdf`
2. **ROS-Industrial `universal_robot` (ROS 1, legacy):** https://github.com/ros-industrial/universal_robot
   (`ur_description/`). The existing `ur5.urdf` was generated from this package. Its
   `melodic-devel`/`noetic-devel` branches include the e-series xacros.
3. **Pre-generated, without ROS:** the `robot_descriptions` Python package
   (`from robot_descriptions import ur10e_description`) downloads a description and exposes `URDF_PATH`.
   Check which upstream repo it pulls and that the inertials are present. MuJoCo Menagerie's
   `universal_robots_ur10e` is MJCF, not URDF, so it is useful only for the web demo.

All of these give nominal kinematics. Each physical robot has its own factory calibration, which
UR's `ur_calibration` tool can extract if you need exact FK for one specific arm.

## How models are found: no code changes needed

- [gtdynamics/config.h.in:30-31](../gtdynamics/config.h.in#L30-L31) sets
  `kUrdfPath = "@PROJECT_SOURCE_DIR@/models/urdfs/"` (and `kSdfPath`) when CMake configures the build.
- [gtdynamics.i:11-12](../gtdynamics.i#L11-L12) makes these available in Python as
  `gtd.URDF_PATH` / `gtd.SDF_PATH`.
- [gtdynamics/universal_robot/sdf.h:30](../gtdynamics/universal_robot/sdf.h#L30):
  `CreateRobotFromFile(path, model_name = "", preserve_fixed_joint = false)` parses URDF and SDF
  (through libsdformat).

These paths point into the source tree, so a file added under `models/urdfs/` can be loaded right
away. You don't need to change CMake, the wrapper or the install rules.

## Directories to update

### 1. `models/urdfs/ur10e/` (required): the model itself
**Purpose:** the shared storage for all robot description files, which C++, Python and examples
reach through `kUrdfPath`.

**Changes:**
- Create `models/urdfs/ur10e/ur10e.urdf` with `visual/` and `collision/` subfolders, following the
  `ur5/` layout.
- Generate the URDF from the official `ur_description` xacro (ROS 2 `Universal_Robots_ROS2_Description`,
  `ur_type:=ur10e`), then **flatten it to plain URDF**. GTDynamics reads plain URDF through sdformat
  and cannot expand xacro.
- **Rewrite mesh paths** from `package://ur_description/meshes/ur10e/...` to relative paths
  (`visual/base.dae`, `collision/base.stl`). `package://` URIs cannot be resolved outside ROS.
- **Check the inertials:** every movable link needs a nonzero mass and inertia, because the dynamics
  factors depend on them. The UR xacros take these from `physical_parameters.yaml`, so make sure
  they were expanded.
- **Remove or simplify ROS-only parts:** `<gazebo>` plugins, `<ros2_control>`, `<transmission>`.
  The UR5 file still has gazebo plugins, which sdformat ignores. Removing them makes the file cleaner.
- **Think about the fixed joints** (`base_link-base_fixed_joint`, `tool0`, `world_joint`, `ft_frame`
  and `flange` on e-series). With the default `preserve_fixed_joint=false`, sdformat merges the links
  attached by fixed joints into their parents. You should end up with the 6 revolute joints
  `shoulder_pan` … `wrist_3`. **Checked:** `preserve_fixed_joint=true` does *not* keep `tool0`,
  `flange`, `ft_frame` or `base`. sdformat drops massless links ("link[ft_frame] is not modeled in
  sdf"), so both modes give 7 links and 6 joints. To use the TCP, apply the constant offset
  `wrist_3_link → tool0` (translation 0, the rotation from the `flange`/`tool0` joints) in code.
- Keep the UR license/attribution header (BSD-3) at the top of the URDF.

**Why:** this is the only change you must make. The rest of the steps check the model or put it to use.

To regenerate these files from the official xacro with ROS 2 instead, see
[URDF_GENERATION_ROS2.md](URDF_GENERATION_ROS2.md).

**Status: done.** `models/urdfs/ur10e/` contains `ur10e.urdf`, `visual/*.dae`, `collision/*.stl` and
`LICENSE`. The source is upstream tag `4.4.0` (commit `81f7c9fe`). The UR10e meshes are BSD-3; UR's
restricted graphics terms cover only other models. The file was converted by hand from the YAML
files because this machine has no ROS or xacro. Upstream `base_link_inertia` is folded into
`base_link`, which leaves the geometry unchanged. Checks run:
- All 14 mesh paths resolve.
- The robot loads with 7 links and 6 joints, with masses from `physical_parameters.yaml`.
- FK of `wrist_3_link` matches the standard UR10e DH model (expressed in `base`) to 3e-10 in full
  pose, at zero and 5 random configurations.

Note for step 2: GTDynamics stores link poses at the **center of mass**. To compare with DH or the
URDF link frame, compose with the inverse of the link's inertial origin. For `wrist_3_link`, that
is `Pose3(Rot3(), Point3(0, 0, 0.026))`.

### 2. `python/tests/` (strongly recommended): Python regression test
**Purpose:** pytest/unittest tests that run against the wrapped `gtdynamics` module.

**Changes:** add `python/tests/test_ur10e.py` and base it on
[python/tests/test_panda.py](../python/tests/test_panda.py):
- Load the model with `Path(gtd.URDF_PATH) / "ur10e" / "ur10e.urdf"`.
- Check the counts: 7 links / 6 joints if fixed joints are merged.
- Check forward kinematics at zero joint angles against the known UR10e geometry
  (d1 = 0.1807, a2 = −0.6127, a3 = −0.57155, d4 = 0.17415, d5 = 0.11985, d6 = 0.11655 m).

**Why:** a test catches mistakes in the URDF (wrong axes, missing inertia, a joint chain that fails
to parse) and guards against regressions. The UR5 model never got such a test.

### 3. `tests/` (optional): C++ test
**Purpose:** C++ unit tests. [tests/CMakeLists.txt](../tests/CMakeLists.txt) picks up every
`test*.cpp` file through `gtsamAddTestsGlob`, so you don't need to change the CMake file.

**Changes (optional):** add `tests/testUR10e.cpp`, which loads the model with
`CreateRobotFromFile(kUrdfPath + std::string("ur10e/ur10e.urdf"))` and checks the link/joint counts
and FK, matching the Python test. You could also add a loader helper to
[gtdynamics/universal_robot/RobotModels.h](../gtdynamics/universal_robot/RobotModels.h) (like
`simple_urdf` and `jumping_robot`) if several C++ tests will share the model.

**Why:** the C++ core is the main API, so a C++ test checks the model without going through the
Python wrapper.

### 4. `examples/` (optional): a showcase
**Purpose:** each subfolder is a demo (C++ executable and/or Python script). Many demos use
`pybullet.loadURDF(gtd.URDF_PATH + ...)` for visualization, for example
[examples/example_a1_walking/sim.py](../examples/example_a1_walking/sim.py).

**Changes (optional):** add `examples/example_ur10e_.../` (for example, IK or trajectory
optimization). A C++ example also needs an entry in [examples/CMakeLists.txt](../examples/CMakeLists.txt).

**Why:** this only shows how to use the model. Skip it unless you have a use case.

### 5. `gtdynamics/<robot>/` (only for robot-specific algorithms)
**Purpose:** robot-specific C++ modules, for example
[gtdynamics/pandarobot/](../gtdynamics/pandarobot/) (IKFast solver + GP prior) and `jumpingrobot/`.

**Changes:** none, unless you add UR10e-specific code such as an analytic IK solver. A new module
would also need entries in [gtdynamics/CMakeLists.txt](../gtdynamics/CMakeLists.txt) and, to be
used from Python, in [gtdynamics.i](../gtdynamics.i).

**Why:** general robots work through `universal_robot/Robot`. A dedicated folder is only worth
having for algorithms specific to one robot.

### 6. `models/mujoco/`, `scripts/`, `webdemo/` (only if the UR10e should appear in the web demo)
**Purpose:** the browser demo loads MuJoCo Menagerie models. The models are fetched by
[scripts/bootstrap_mujoco_models.py](../scripts/bootstrap_mujoco_models.py) and are not committed.
Per-robot configs live in `webdemo/public/examples/configs/*.json` and motions in `.../motions/`.

**Changes (optional):** add `universal_robots_ur10e` to the bootstrap script's list of models, add
the symlink to `.gitignore` (next to the existing `models/mujoco/unitree_a1` lines), and add
`webdemo/public/examples/configs/ur10e.json` plus a motions folder. Update
[models/mujoco/README.md](../models/mujoco/README.md) to list the new model.

**Why:** the web demo works separately from the URDF path. It is needed only to show the UR10e in
the browser.

## Directories that need no changes
- `cmake/`, the root `CMakeLists.txt`, `gtdynamics.i`, `gtdynamics/config.h.in`: the model paths
  are already set up for the whole directory.
- `python/gtdynamics/`: the Python package picks up `URDF_PATH` without changes.

## Verification
1. Rebuild (`cmake --build build`) so that the Python wrapper is current. Adding a URDF alone doesn't
   need a rebuild; a new C++ test does.
2. Quick check:
   `python -c "import gtdynamics as gtd; r=gtd.CreateRobotFromFile(gtd.URDF_PATH+'ur10e/ur10e.urdf'); r.print()"`.
   Make sure there are 6 revolute joints with sensible axes and masses.
3. `pytest python/tests/test_ur10e.py`, and, if you added it, `ctest -R testUR10e` (or `make testUR10e.run`).
4. Optional visual check: open the model with `pybullet.loadURDF` to confirm that the meshes resolve.

# Generating the UR10e URDF with ROS 2 (xacro)

This guide regenerates `models/urdfs/ur10e/` (`ur10e.urdf`, `visual/`, `collision/`, `LICENSE`)
from the official Universal Robots xacro. It replaces the hand-converted URDF made for step 1 of
[UR10E_INTEGRATION_PLAN.md](UR10E_INTEGRATION_PLAN.md), which was written without a ROS install.

## Prerequisites

- **OS:** Ubuntu 24.04 (noble), which pairs with **ROS 2 Jazzy**.
- **Source:** [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
  release **4.4.0**, commit `81f7c9fe003c2f0e02eec5a7253f1b262ad38b61`. It is built from source
  because the apt package `ros-jazzy-ur-description` may not be at 4.4.0.
- **Shells:** you can load ROS and the GTDynamics `.venv` in the same shell. ROS Jazzy's Python
  packages don't overlap with the venv's (numpy, gtsam, gtdynamics, pybullet), and it ships no
  gtsam, sdformat or boost libraries. If a venv import ever fails, try again in a shell without ROS
  sourced.
- Paths assume the repo is at `~/repos/GTDynamics`. If it isn't, change `OUT` in step 3.

## 1. Install ROS 2 Jazzy (minimal) and xacro

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe

# Add the ROS 2 apt repository (official ros2-apt-source method)
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt install -y ros-jazzy-ros-base ros-jazzy-xacro ros-dev-tools liburdfdom-tools
```

What each package provides:

| Package | Provides |
|---|---|
| `ros-jazzy-ros-base` | `ros2` CLI and the ament environment |
| `ros-jazzy-xacro` | `xacro` (flattens xacro into plain URDF) |
| `ros-dev-tools` | `colcon` |
| `liburdfdom-tools` | `check_urdf` |

## 2. Build `ur_description` at release 4.4.0

```bash
mkdir -p ~/repos/ur_ws/src && cd ~/repos/ur_ws/src
git clone --branch 4.4.0 --depth 1 \
  https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
git -C Universal_Robots_ROS2_Description rev-parse HEAD   # expect 81f7c9fe003c2f0e02eec5a7253f1b262ad38b61

cd ~/repos/ur_ws
# source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur_description
source install/setup.bash
ros2 pkg prefix ur_description    # should print ~/ur_ws/install/ur_description
```

**`ros2: command not found`:** ROS isn't loaded in this shell. Adding `/opt/ros/jazzy/bin` to
`PATH` alone isn't enough, because `ros2` also needs the `PYTHONPATH`, `AMENT_PREFIX_PATH` and
`LD_LIBRARY_PATH` that `setup.bash` sets. To load ROS in every terminal, add this to `~/.bashrc`:

```bash
# ROS 2 Jazzy (+ UR description workspace if built)
source /opt/ros/jazzy/setup.bash
[ -f ~/repos/ur_ws/install/setup.bash ] && source ~/repos/ur_ws/install/setup.bash
```

Then open a new terminal, or run `source ~/.bashrc`. The `[ -f ]` guard keeps new shells working
before `colcon build` has created the workspace.

Generating the URDF only needs `xacro`. The RViz viewer needs the package's other runtime
dependencies, which you can install with:

```bash
sudo rosdep init; rosdep update
rosdep install --from-paths ~/repos/ur_ws/src -y --ignore-src
ros2 launch ur_description view_ur.launch.py ur_type:=ur10e
```

## 3. Generate the URDF, `visual/` and `collision/`

The 4.4.0 top-level `urdf/ur.urdf.xacro` already adds a fixed `world` link, and its output has no
`ros2_control`, transmission or gazebo blocks. The only cleanup needed is rewriting the mesh URIs.

```bash
source /opt/ros/jazzy/setup.bash && source ~/ur_ws/install/setup.bash
UR=$(ros2 pkg prefix ur_description)/share/ur_description
OUT=~/repos/GTDynamics/models/urdfs/ur10e

# Keep the hand-written version for comparison (skip if you don't want it)
[ -f $OUT/ur10e.urdf ] && mv $OUT/ur10e.urdf $OUT/ur10e_handwritten.urdf

mkdir -p $OUT/visual $OUT/collision

# 1) Flatten the xacro to plain URDF
xacro $UR/urdf/ur.urdf.xacro ur_type:=ur10e name:=ur10e > $OUT/ur10e.urdf

# 2) Rewrite package:// mesh URIs to paths relative to the URDF (same as ur5/)
sed -i 's#package://ur_description/meshes/ur10e/##g' $OUT/ur10e.urdf
```
- strips the ROS-specific prefix off every mesh path in the generated URDF, so that GTDynamics can find the mesh files without ROS.

What xacro writes:
```xml
<mesh filename="package://ur_description/meshes/ur10e/visual/base.dae"/>
<mesh filename="package://ur_description/meshes/ur10e/collision/base.stl"/>
```
After the sed command:
```xml
<mesh filename="visual/base.dae"/>
<mesh filename="collision/base.stl"/>
```


```bash

# 3) Record where the file came from (inserted after the <?xml?> line)
sed -i '1a <!-- Generated by xacro from Universal_Robots_ROS2_Description 4.4.0 (81f7c9fe), ur_type:=ur10e. Mesh paths made relative. BSD-3-Clause, see LICENSE. -->' $OUT/ur10e.urdf

# 4) Copy the meshes and the license
cp $UR/meshes/ur10e/visual/*.dae    $OUT/visual/
cp $UR/meshes/ur10e/collision/*.stl $OUT/collision/
cp ~/ur_ws/src/Universal_Robots_ROS2_Description/LICENSE $OUT/
```

Why the mesh rewrite is needed: GTDynamics loads URDFs through libsdformat, which can't resolve
`package://` URIs without ROS. Relative paths are resolved from the URDF's directory, which is
how `models/urdfs/ur5/` and `models/urdfs/panda/` work.

## 4. Check the URDF

Still in the ROS shell (or any shell with `liburdfdom-tools`):

```bash
# The URDF is well-formed and the tree is world -> base_link -> ... -> tool0
check_urdf $OUT/ur10e.urdf

# No package:// URIs left, and every mesh path exists
grep -c "package://" $OUT/ur10e.urdf          # expect 0
grep -o 'filename="[^"]*"' $OUT/ur10e.urdf | cut -d'"' -f2 | sort -u | \
  while read f; do [ -f "$OUT/$f" ] && echo "ok  $f" || echo "MISSING $f"; done
```

## 5. Check the model in GTDynamics

Open a **new terminal with no ROS sourced** and activate the repo `.venv`:

```bash
cd ~/repos/GTDynamics
python -c "import gtdynamics as gtd; r=gtd.CreateRobotFromFile(gtd.URDF_PATH+'ur10e/ur10e.urdf'); print(len(r.links()),'links',len(r.joints()),'joints')"
# expect: 7 links 6 joints
```

### FK regression check against the UR10e DH parameters

Save this as `check_ur10e.py` somewhere outside the repo and run it with the `.venv` Python. It
compares the GTDynamics FK of `wrist_3_link` with UR's published DH model at zero joint angles and
at 5 random configurations. On the hand-written URDF, the maximum error was about **3e-10**.

```python
import numpy as np
from gtsam import Values, Pose3, Rot3, Point3
import gtdynamics as gtd

robot = gtd.CreateRobotFromFile(gtd.URDF_PATH + "ur10e/ur10e.urdf")
print("links:", [(l.name(), round(l.mass(), 3)) for l in robot.links()])
print("joints:", [j.name() for j in robot.joints()])

# Standard UR10e DH parameters.
d = [0.1807, 0, 0, 0.17415, 0.11985, 0.11655]
a = [0, -0.6127, -0.57155, 0, 0, 0]
alpha = [np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0]
def dh(q):
    T = np.eye(4)
    for i in range(6):
        ct, st, ca, sa = np.cos(q[i]), np.sin(q[i]), np.cos(alpha[i]), np.sin(alpha[i])
        T = T @ np.array([[ct, -st * ca, st * sa, a[i] * ct],
                          [st, ct * ca, -ct * sa, a[i] * st],
                          [0, sa, ca, d[i]], [0, 0, 0, 1]])
    return T

# The DH frames are expressed in 'base', which is base_link rotated by pi about z.
# wrist_3_link coincides with DH frame 6.
Rz = np.diag([-1, -1, 1, 1.0])
rng = np.random.default_rng(0)
worst = 0
for q in [np.zeros(6)] + [rng.uniform(-np.pi, np.pi, 6) for _ in range(5)]:
    vals = Values()
    for i, n in enumerate(["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                           "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]):
        gtd.InsertJointAngle(vals, robot.joint(n).id(), q[i])
    fk = robot.forwardKinematics(vals, 0, "base_link")
    w3 = robot.link("wrist_3_link")
    # GTDynamics poses are COM poses; undo wrist_3's inertial origin (0, 0, -0.026).
    bTw3 = gtd.Pose(fk, w3.id()).compose(Pose3(Rot3(), Point3(0, 0, 0.026))).matrix()
    expected = Rz @ dh(q)
    worst = max(worst, np.abs(bTw3 - expected).max())
    if not q.any():
        print("zero-config wrist_3 position:", bTw3[:3, 3].round(5), "expected", expected[:3, 3].round(5))
print("max |T_gtd - T_dh| over 6 configs:", worst)
```

**Center-of-mass convention:** GTDynamics stores each link's pose at its center of mass, not at
the URDF link frame. To compare against DH or URDF frames, compose with the inverse of the link's
`<inertial><origin>`. For `wrist_3_link` that is `Pose3(Rot3(), Point3(0, 0, 0.026))`. If you get
all-zero positions at zero configuration, you composed with `bMcom()` instead: `bMcom()` is the
link's zero-configuration COM pose, not the offset from the link frame.

## Differences from the hand-written URDF

- **`base_link_inertia` and `base_joint` are kept.** The xacro output has an empty `base_link`, a
  fixed child `base_link_inertia` (rotated by π about z) that carries the base mesh and inertia, and
  a fixed `base_joint` from `world`. When GTDynamics loads the file (`preserve_fixed_joint=false`),
  libsdformat merges `base_link_inertia` into `base_link`. The joints, masses and FK should match
  the hand-written file; the FK check above confirms this. The hand-written file folds this
  rotation into the `shoulder_pan_joint` origin.
- **Header:** the generated file has xacro's generic "autogenerated" banner. The `sed` comment in
  step 3 records the source.
- **Massless frames are dropped either way.** `tool0`, `flange`, `ft_frame` and `base` have no
  inertia, so libsdformat drops them even with `preserve_fixed_joint=true` (warning "link[ft_frame]
  is not modeled in sdf"). Both modes give 7 links and 6 joints. To use the TCP, apply the constant
  `wrist_3_link → tool0` transform in code.

## Adapting to other UR models

Change `ur_type:=ur10e` and the `meshes/ur10e/` folder in the `sed` and `cp` commands. The model
names are listed in `urdf/ur.urdf.xacro` (ur3, ur5, ur10, ur3e, ur5e, ur7e, ur10e, ur12e, ur16e,
ur8long, ur15, ur18, ur20, ur30, ur10g-1750, ur17g-1300, ur18g-950).

**License warning:** the UR8LONG, UR15, UR18, UR20, UR30, g-series and Smart Panel meshes are *not*
BSD-3. UR's "Terms and Conditions for use of Graphical Documentation" cover them (see the upstream
README). Check those terms before committing those meshes. The e-series and CB3 meshes, including
the UR10e, are BSD-3.

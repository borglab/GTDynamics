# GTDynamics Example: Kinematic Motion Planning

This executable implements a simple controller that uses the robot's kinematics and a pre-defined base trajectory to compute the joint angles along the trajectory and reach them using a PD controller. The resulting controller is inherently unstable due to two main reasons:

1. The controller does not consider the robot's dynamics and thus does not satisfy the constraints required to achieve balance at the base and prevent slippage at the contacts.
2. The resulting controller is an open-loop controller and thus does not correct for the positional error that builds up due to unmodeled inputs to the system (e.g. slippage, moments about the CoM, etc.).

## Running the example:

**1. Optimizing for the joint angles.**

```
mkdir build; cd build
cmake ../
make
./exec
```

**2. Visualizing the trajectory (_requires Matlab_)**

- You will need to copy the generated `traj.csv` file in the example build directory to this directory.

- Open Matlab and navigate to this example's directory. In the Matlab command window, run:

`>> visualize_traj`

**3. Simulating the controller (_requires Python3 and PyBullet_)**

First execute step 1.

Install `PyBullet` if you don't already have it with:

```bash
pip install pybullet
```

Run this script from the command line:

```bash
python kinematic_mp_sim.py
```

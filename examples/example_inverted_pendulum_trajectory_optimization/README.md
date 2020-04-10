# Simple Trajectory Optimization: Inverted Pendulum

This example executable applies trajectory optimization with GTDynamics to the inverted pendulum system. No prior knowledge of the system's dynamics must be specified. Instead, we specify the robot's kinematic configuration (via the `inverted_pendulum.urdf` file), the system's initial conditions, objectives for the terminal state, and some path objectives. GTDynamics handles the rest!

## Running the example:

**1. Generating the trajectory.**
```
mkdir build; cd build
cmake ../
make
./exec
```

**Note:** Set `CMAKE_PREFIX_PATH` to the path to your GTDynamics install if necessary.

**2. Visualizing the trajectory (_requires MatLab_)**

Open MatLab and navigate to this example's directory. In the MatLab command window, run:

```>> visualize```
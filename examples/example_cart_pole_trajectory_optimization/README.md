# Simple Trajectory Optimization: Cart Pole

This example executable applies trajectory optimization with GTDynamics to the cart pole system. No prior knowledge of the system's dynamics must be specified. Instead, we specify the robot's kinematic configuration (via the `cart_pole.urdf` file), the system's initial conditions, objectives for the terminal state, and some path objectives. GTDynamics handles the rest!

## Running the example:

**1. Generating the trajectory.**
```
make example_cart_pole_trajectory_optimization.run
```

**Note:** Set `CMAKE_PREFIX_PATH` to the path to your GTDynamics install if necessary.

**2. Visualizing the trajectory (_requires Matlab_)**

Open Matlab and navigate to this example's directory. In the Matlab command window, run:

```>> visualize```

OR run python [visualize.py](visualize.py) and give the `traj.csv` filename as an argument.
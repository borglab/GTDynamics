# Cable Robot

This folder contains code pertinent to cable robot simulation and control.

Make sure to set the `GTDYNAMICS_BUILD_CABLE_ROBOT` cmake flag to `ON` to build these codes.  For example,

`cmake -DGTDYNAMICS_BUILD_CABLE_ROBOT=ON ..`

## Variable name / notation conventions

Cable robot specific notations:
  * x - the end effector Pose
  * a - the positions of the cable mounting points on the frame
  * b - the positions of the cable mounting points on the end-effector
  * l/ldot/lddot - the cable length and its time derivatives
  * t - cable tension / torque (sometimes t is used for time, but I try to use k for time instead)

General notations:
  * w - the world/fixed frame
  * xTy - the pose of y in x's frame
  * xPy - the position of y in x's frame
  * xRy - the rotation of y in x's frame
  * Vx - the twist of x in its own frame
  * VAx - the twist acceleration of x in its own frame
  * Fx - the wrench acting on x in its own frame
  * x_H_y - the partial derivative (jacobian) of x w.r.t. y

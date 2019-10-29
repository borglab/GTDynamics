# dynamics
Dynamics with factor graphs
===================================================
Vrep Simulation
-----
To use the remote API functionanlity in Python script, you will need following 3 items, which are located in V-REP's installation directory, under programming/remoteApiBindings/python:
- vrep.py
- vrepConst.py
- remoteApi.so, remoteApi.dylib, remoteApi.dll (depending on your target platform)

# Using the wrapper in MATLAB

Open MATLAB and add the `gtsam_toolbox` to the MATLAB path (by default, cmake prefix is /usr/local, change it to your prefix if you customized it)

```matlab
addpath('/usr/local/gtsam_toolbox')
```


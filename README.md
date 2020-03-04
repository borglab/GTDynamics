# GTDynamics

### *Full kinodynamics constraints for arbitrary robot configurations with factor graphs.*
<!-- =================================================== -->

![Build Status](https://travis-ci.com/Alescontrela/GTDynamics.svg?token=V6isP7NT7qX4qsBuX1sY&branch=master)

GTDynamics is a library that allows the user to express the full kinodynamics constraints of an arbitrary robot configuration on a factor graph. These constraints can be used to solve the forward and inverse dynamics problems.

## Dependencies

* [`GTSAM4`](https://github.com/borglab/gtsam)

### macOS
* [`sdformat8`](https://bitbucket.org/osrf/sdformat/src/default/)
```bash
$ # Install homebrew.
$ ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
$ # Download sdformat to your preferred location.
$ brew tap osrf/simulation
$ brew install sdformat8
```

### Ubuntu
* [`sdformat8`](https://bitbucket.org/osrf/sdformat/src/default/)
```bash
$ # Install homebrew (for linux).
$ sh -c "$(curl -fsSL https://raw.githubusercontent.com/Linuxbrew/install/master/install.sh)"
$ # Download sdformat to your preferred location.
$ brew tap osrf/simulation
$ brew install sdformat8
```

If issues arise when installing dependencies for sdformat, reference the following [tutorial](http://gazebosim.org/tutorials?tut=install_dependencies_from_source) to install sdformat from source.

## Installing
```bash
$ git clone https://github.com/borglab/GTDynamics.git
$ cd GTDynamics
$ mkdir build; cd build
# Optionally specify install path with -DCMAKE_INSTALL_PREFIX
$ cmake ../
$ make
$ sudo make install
```

## Running tests

```bash
$ make check
```

## Running examples

The `/examples` directory contains example projects that demonstrate how to include GTDynamics in your application. To run an example, ensure that the `CMAKE_PREFIX_PATH` is set to the GTDynamics install directory.

1. Navigate to the example's subdirectory and create a build directory. e.g.
```bash
cd GTDynamics/examples/example_forward_dynamics
mkdir build; cd build
```

2. Make the example.

If GTDynamics was installed to `~/JohnDoe/gtdynamics_install`, then run the cmake command with:

```bash
cmake -DCMAKE_PREFIX_PATH=~/JohnDoe/gtdynamics_install
make
```

3. Run the example!
```bash
./exec
```

## Citing this work

The core paper behind this work is:
```
@misc{1911.10065,
    Author = {Mandy Xie and Frank Dellaert},
    Title = {A Unified Method for Solving Inverse, Forward, and Hybrid Robot Dynamics using Factor Graphs},
    Year = {2019},
    Eprint = {arXiv:1911.10065},
}
```


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

## Running Tests

```bash
$ make check
```

## Running Examples

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

## Python Wrapper

GTDynamics now supports a Pybind11-based Python API.

To start, please download and install the [GTwrap repository](https://github.com/borglab/wrap).

To compile and install the GTDynamics python library:

1. In the build directory, run `cmake` with the flag `GTDYNAMICS_BUILD_PYTHON=ON`.

    ```sh
    cmake -DGTDYNAMICS_BUILD_PYTHON ..
    ```

2. Build as normal and install the python package.

    ```sh
    make && make python-install
    ```

## Citing This Work

Please cite the following paper if you use this code as part of any published research:

```
@misc{2011.06194,
    Author = {Mandy Xie, Alejandro Escontrela, and Frank Dellaert},
    Title = {A Factor-Graph Approach for Optimization Problems with Dynamics Constraints},
    Year = {2020},
    Eprint = {arXiv:2011.06194},
}
```


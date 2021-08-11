# GTDynamics

**<span style=“color:red”>This library is still under very active development, hence bleeding edge, and not "supported" in the way [GTSAM](https://gtsam.org) is. In particular, we are still actively re-factoring the way we deal with time and time intervals.</span>**


### *Full kinodynamics constraints for arbitrary robot configurations with factor graphs.*
<!-- =================================================== -->

![Build Status](https://travis-ci.com/Alescontrela/GTDynamics.svg?token=V6isP7NT7qX4qsBuX1sY&branch=master)

GTDynamics is a library that allows the user to express the full kinodynamics constraints of an arbitrary robot configuration on a factor graph. These constraints can be used to solve the forward and inverse dynamics problems.

## Dependencies

* [GTSAM4](https://github.com/borglab/gtsam)
* [gtwrap](https://github.com/borglab/wrap)
* [sdformat10](https://github.com/osrf/sdformat)

## Installing SDFormat

GTDynamics uses the SDFormat parser to parse SDF/URDF files containing robot descriptions.

### Homebrew

If you are on a Mac, using Homebrew is the easiest way to get SDFormat installed and it also makes switching versions straightforward.

```sh
$ # Install homebrew.
$ /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
$ # Download sdformat to your preferred location.
$ brew tap osrf/simulation
$ brew install sdformat10
```

### Source

Alternatively, you can install from source if you are on Linux or want more fine-tuned control.

We provide an Ubuntu-based process below. Please reference [this tutorial](http://gazebosim.org/tutorials?tut=install_dependencies_from_source) for complete details on installing from source.


```sh
# Install basic dependencies
sudo apt-get install ruby-dev build-essential libtinyxml-dev libboost-all-dev cmake pkg-config

# sdformat requires libignition-math
sudo apt-get install libignition-math4-dev

# Set the version to install
export GTD_SDFormat_VERSION="10.5.0"

# Download specific version of SDFormat
wget http://osrf-distributions.s3.amazonaws.com/sdformat/releases/sdformat-${GTD_SDFormat_VERSION}.tar.bz2

tar -xvjf sdformat-${GTD_SDFormat_VERSION}.tar.bz2

cd sdformat-${GTD_SDFormat_VERSION}
mkdir build && cd build

cmake -DCMAKE_INSTALL_PREFIX ../install ..
make -j4
sudo make install
```

## Installing GTDynamics
```sh
$ git clone https://github.com/borglab/GTDynamics.git
$ cd GTDynamics
$ mkdir build; cd build
# We can specify the install path with -DCMAKE_INSTALL_PREFIX
$ cmake -DCMAKE_INSTALL_PREFIX=../install ..
$ make
$ sudo make install
```

## Running Tests

```sh
$ make check
```

## Running Examples

The `/examples` directory contains example projects that demonstrate how to include GTDynamics in your application. To run an example, ensure that the `CMAKE_PREFIX_PATH` is set to the GTDynamics install directory.

1. Navigate to the example's subdirectory and create a build directory. e.g.
```sh
cd GTDynamics/examples/example_forward_dynamics
mkdir build; cd build
```

2. Make the example.

If GTDynamics was installed to `~/JohnDoe/gtdynamics_install`, then run the cmake command with:

```sh
cmake -DCMAKE_PREFIX_PATH=~/JohnDoe/gtdynamics_install ..
make
```

3. Run the example!
```sh
./exec
```

## Python Wrapper

GTDynamics now supports a Pybind11-based Python API.

To start, please download and install the [GTwrap repository](https://github.com/borglab/wrap).

To compile and install the GTDynamics python library:

1. In the build directory, run `cmake` with the flag `GTDYNAMICS_BUILD_PYTHON=ON`.

    ```sh
    cmake -DGTDYNAMICS_BUILD_PYTHON=ON ..
    ```

2. Build as normal and install the python package.

    ```sh
    make && make python-install
    ```

3. To run the Python tests, you can simply run:

    ```sh
    make python-test
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


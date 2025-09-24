# GTDynamics

**<span style=“color:red”>This library is still under very active development, hence bleeding edge, and not "supported" in the way [GTSAM](https://gtsam.org) is. In particular, we are still actively re-factoring the way we deal with time and time intervals.</span>**


### *Full kinodynamics constraints for arbitrary robot configurations with factor graphs.*

![Build Status](https://travis-ci.com/Alescontrela/GTDynamics.svg?token=V6isP7NT7qX4qsBuX1sY&branch=master)

GTDynamics is a library that allows the user to express the full kinodynamics constraints of an arbitrary robot configuration on a factor graph. These constraints can be used to solve the forward and inverse dynamics problems.

## Dependencies

* [GTSAM4](https://github.com/borglab/gtsam)
* [gtwrap](https://github.com/borglab/wrap)
* [sdformat13](https://github.com/osrf/sdformat)

## Installing SDFormat

GTDynamics uses the SDFormat parser to parse SDF/URDF files containing robot descriptions.

### Homebrew

Using Homebrew is the easiest way to get SDFormat installed and it also makes switching versions straightforward.

```sh
$ # Install homebrew.
$ /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
$ # Set up the tap ind install sdformat13
$ brew tap osrf/simulation
$ brew install sdformat13
```

### Source

Alternatively, you can install from source if you are on Linux or want more fine-tuned control.

We provide an Ubuntu-based process below. Please reference [this tutorial](http://gazebosim.org/tutorials?tut=install_dependencies_from_source) for complete details on installing from source.


```sh
# Install basic dependencies
sudo apt-get install -y ruby-dev build-essential libboost-all-dev cmake pkg-config wget lsb-release

# Setup the repo
sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update

# Install SDFormat dependencies
sudo apt-get install -y libtinyxml2-dev liburdfdom-dev libignition-cmake2-dev libignition-tools-dev libignition-math6-dev

# Set the version to install
export GTD_SDFormat_VERSION="10.5.0"

# Download specific version of SDFormat
wget http://osrf-distributions.s3.amazonaws.com/sdformat/releases/sdformat-${GTD_SDFormat_VERSION}.tar.bz2

tar -xvjf sdformat-${GTD_SDFormat_VERSION}.tar.bz2

cd sdformat-${GTD_SDFormat_VERSION}
mkdir build && cd build

cmake -DCMAKE_INSTALL_PREFIX=../install ..
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

The `examples` directory contains various full example projects demonstrating the use of GTDynamics for various robotic applications.

We recommend going through the examples to get a better understanding of how to use GTDynamics for your own use cases.

*NOTE* The examples are made to run within the GTDynamics source folder and are not standalone projects.

Run these examples with
```sh
$ make example_XXX.run
```
where `XXX` corresponds to a folder name in `examples`.  For example, `make example_forward_dynamics.run`.

## Including GTDynamics With CMake

The `examples/cmake_project_example` directory contains an example CMake-based project that demonstrates how to include GTDynamics in your application.

Use this as a template when you want to set up your own project that uses GTDynamics (e.g. separate git repo, ROS, personal libraries, etc).

To build the project:

1. Navigate to the example's subdirectory and create a build directory. e.g.
    ```sh
    cd GTDynamics/examples/cmake_project_example
    mkdir build; cd build
    ```

2. Make the example.

    If GTDynamics was installed to `~/JohnDoe/GTDynamics/install`, then run the cmake command with:

    ```sh
    cmake -DCMAKE_PREFIX_PATH=~/JohnDoe/GTDynamics/install ..
    make
    ```

3. Run the example!
    ```sh
    ./example
    ```

## Python Wrapper (Recommended use case)

GTDynamics now supports a Pybind11-based Python API.

GTWrap comes bundled with GTSAM, which generates a corresponding GTSAM Python API. The same GTWrap package can be used to generate python bindings for GTDynamics (i.e. it is not necessary to manually install a separate GTWrap).

**Note**: when using CMake, it is ideal for GTSAM and GTDynamics to have the same, *non* `/usr/local` prefix for installing packages. To update the CMake prefix from a system directory, use the flag `CMAKE_INSTALL_PREFIX=/path/to/install/dir` when running `cmake`.

To compile and install the GTDynamics python library:

1. Ensure that GTSAM is built with generated python bindings. If not, go to the build directory and run `cmake` with the flag `GTSAM_BUILD_PYTHON=ON`. It is highly advised to specify a *non* `user/local` CMake prefix for installing packages. Afterwards, install the GTSAM python package.

    ```sh
    cmake -DGTSAM_BUILD_PYTHON=ON -DCMAKE_INSTALL_PREFIX=/path/to/install/dir ..
    make && make install && make python-install
    ```

2. In the GTDynamics build directory, run `cmake` with the flag `GTDYNAMICS_BUILD_PYTHON=ON`. It is highly advised for the GTDynamics CMake prefix to match the CMake prefix used for GTSAM. Again, use the `CMAKE_INSTALL_PREFIX=/path/to/install/dir` flag to specify the updated prefix.

    ```sh
    cmake -DGTDYNAMICS_BUILD_PYTHON=ON -DCMAKE_INSTALL_PREFIX=/path/to/install/dir ..
    ```

3. Build as normal and install the python package.

    ```sh
    make && make python-install
    ```

4. To run the Python tests, you can simply run:

    ```sh
    make python-test
    ```

    You can also run individual test suites, e.g. with:

    ```sh
    make python-test.base
    make python-test.cablerobot
    ```

## (Alternative) Python Wrapper installation

If preferred, GTWrap can be [downloaded and installed separately](https://github.com/borglab/wrap). Afterwards, follow the instructions above from step 2 for building and installing the GTDynamics python bindings.

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


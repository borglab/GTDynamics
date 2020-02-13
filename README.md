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
$ cmake ../
$ make
$ sudo make install
```

## Running tests

```bash
$ make check
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

## Including this in your project



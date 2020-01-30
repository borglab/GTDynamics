# GTDynamics

### Full kinodynamics constraints for arbitrary robot configurations with factor graphs.
<!-- =================================================== -->

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

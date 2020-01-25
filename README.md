# GTdynamics

### Full kinodynamics motion planning for arbitrary robot configurations with factor graphs.
<!-- =================================================== -->

GTdynamics is a library which allows the user to express the full kinodynamics constraints of an arbitrary robot configuration on a factor graph. Given an initial pose, these constraints - along with additional objective functions - can be used to pose a trajectory optimization problem.

## Dependencies

### macOS
* [`GTSAM4`](https://github.com/borglab/gtsam)
* [`sdformat8`](https://bitbucket.org/osrf/sdformat/src/default/)
```
$ # Install homebrew.
$ ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
$ # Download sdformat to your preferred location.
$ brew tap osrf/simulation
$ brew install sdformat8
```

## Installing
```
git clone <REPO_URL>
cd gtdynamics
mkdir build; cd build
cmake ../
make
sudo make install
```

## Running tests

```
make check
```
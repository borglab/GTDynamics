# GTDynamics Python Wrapper

This directory is where the Pybind11-generated GTDynamics package lives and where the CI jobs assemble and verify the Python bindings.

## Layout
- `gtdynamics/` contains the Python package that mirrors the C++ API and exposes convenience helpers such as the `sim.py` sensor models.
- `templates/` and `Setup.py.in` drive the `pip install .` step that packages the compiled `.so`/`.pyd` into a wheel.
- `tests/` is the unit-test tree run by `make python-test.*` targets defined in `python/CMakeLists.txt`.

## Build prerequisites
1. **GTSAM** must be built with Python support (i.e., `-DGTSAM_BUILD_PYTHON=ON`) and installed to a prefix that GTDynamics can see via `GTSAM_DIR` or `CMAKE_PREFIX_PATH`.
2. **Python tooling**: the CI job installs `setuptools<70`, `wheel`, `numpy`, `pyparsing`, `pyyaml`, and `pybind11-stubgen` before configuring the project; matching this list locally avoids the same runtime issues.
3. On macOS, the workflow creates and activates a virtual environment (`pythonX -m venv venv`) so that `pip install` and the tests run in the same interpreter that baked the bindings.

## Building and installing locally
1. Create a build directory and configure with the wrapper flag enabled:
   ```sh
   mkdir -p build && cd build
   cmake -DGTDYNAMICS_BUILD_PYTHON=ON -DGTSAM_DIR=/path/to/gtsam ..
   ```
   Make sure the Python interpreter you pass to CMake has the same version that your `pip` commands will target.
2. Build the project and the python wrapper target:
   ```sh
   make -j$(nproc)
   ```
3. Install the package into the active Python environment (this is what both the CI job and `python-install` do):
   ```sh
   make python-install
   ```
   This runs `${PYTHON_EXECUTABLE} -m pip install .` in `build/python`, which produces a wheel in `pip`'s cache before installing it.

## Running Python tests
- `make python-test` runs every suite captured by the `PYTHON_UNIT_TEST_SUITE` macro (base plus optional `cablerobot`/`jumpingrobot`).
- Individual suites can be executed via `make python-test.base`, `make python-test.cablerobot`, etc., and they rely on the package being built and discoverable via `PYTHONPATH`.
- The CI job runs the tests on macOS with `venv` activation and on Linux after `sudo make python-install` so you can copy those steps when reproducing test failures.

## Packaging tips
- `python/templates/setup.py.in` reads the CMake-generated `requirements.txt` and packages the shared library blobs (`.so` / `.pyd`) from `python/gtdynamics` so running `pip wheel .` in `build/python` yields a complete asset.
- Keep `python/requirements.txt` in sync with the requirements file copied to `build/python/requirements.txt` so that CI and a local `pip install` use the same dependency list.
- If you need to publish a wheel manually, the packaged wheel that `pip install .` writes to `~/.cache/pip` already encodes the GTDynamics version reported by `CMakeLists.txt`.

## Wheels
### CI wheel pipeline
The Common CI workflow (`.github/workflows/common-ci.yml`) performs the exact steps above (configure with `GTDYNAMICS_BUILD_PYTHON=ON`, build, then `make python-install`) across Ubuntu and macOS runners. That workflow serves as the basis for every wheel that leaves the repo:
- **Develop wheels** mirror whatever commit triggered the workflow (typically a push or a PR targeting the `develop` integration branch) so that nightly or pre-release users get the latest patches.
- **Production wheels** are generated from an official release branch or tag (e.g., `main`/`master` after a version bump). The same commands run, but the resulting wheel is considered release-quality and is the artifact uploaded to PyPI.

### Cleanup script (`.github/scripts/python_wheels/cleanup_gtsam_develop.sh`)
Before building a new develop wheel locally or rerunning the develop CI job, execute the cleanup script from the repository root:
```sh
bash .github/scripts/python_wheels/cleanup_gtsam_develop.sh
```
This removes stale `gtsam` develop wheel artifacts and/or cached build output so the subsequent `pip install`/`make python-install` matches the clean state CI assumes.

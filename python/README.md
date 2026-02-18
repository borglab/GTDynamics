# GTDynamics Python Wrapper

This directory is where the Pybind11-generated GTDynamics package lives and where the CI jobs assemble and verify the Python bindings.

## Layout

- `gtdynamics/` contains the Python package that mirrors the C++ API and exposes convenience helpers such as the `sim.py` sensor models.
- `templates/` and `pyproject.toml` drive the `pip install .` step that packages the compiled `.so`/`.pyd` into a wheel.
- `tests/` is the unit-test tree run by `make python-test.*` targets defined in `python/CMakeLists.txt`.

## Build prerequisites

1. **GTSAM** must be built with Python support (i.e., `-DGTSAM_BUILD_PYTHON=ON`) and installed to a prefix that GTDynamics can see via `GTSAM_DIR` or `CMAKE_PREFIX_PATH`.
2. **Python tooling**: the CI job installs `setuptools<70`, `wheel`, `numpy`, `pyparsing`, `pyyaml`, and `pybind11-stubgen` before configuring the project; matching this list locally avoids the same runtime issues. `pybind11-stubgen` is required for the `python-stubs` CMake target.
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
   This runs `${PYTHON_EXECUTABLE} -m pip install .` in `build/python`, which produces a wheel in `pip`'s cache before installing it. On non-Windows platforms, `python-install` depends on `python-stubs`, so `.pyi` files are generated first.

## GTSAM Python compatibility (important)

Use a `gtsam` Python package built from the same GTSAM install/prefix that GTDynamics links against.
Mixing a local GTDynamics build with an unrelated pip/conda `gtsam` wheel can cause hard runtime failures (for example, process aborts when adding factors to a graph).

If you built GTSAM from source in a sibling repo, prepend it before importing:

```sh
export PYTHONPATH=/path/to/gtsam/build/python:/path/to/GTDynamics/build/python:$PYTHONPATH
```

## Generating type stubs

- Run `make python-stubs` to generate stubs with `pybind11-stubgen`.
- Stubs are generated in `build/python/gtdynamics/*.pyi`.
- The generated `gtdynamics.pyi` file is what tools like Pylance use for attribute completion/type checking on wrapped symbols.

### VS Code / Pylance configuration

If your runtime works but Pylance still reports unknown attributes (for example, `CreateRobotFromFile`), make sure VS Code analyzes the build package path.

In workspace `.vscode/settings.json`:

```json
{
  "python.defaultInterpreterPath": "/path/to/your/python",
  "python.analysis.extraPaths": [
    "${workspaceFolder}/build/python"
  ]
}
```

Then run:

```sh
make python-stubs
```

and reload the VS Code window.

## Running Python tests

- `make python-test` runs every suite captured by the `PYTHON_UNIT_TEST_SUITE` macro (base plus optional `cablerobot`/`jumpingrobot`).
- Individual suites can be executed via `make python-test.base`, `make python-test.cablerobot`, etc., and they rely on the package being built and discoverable via `PYTHONPATH`.
- The CI job runs the tests on macOS with `venv` activation and on Linux after `sudo make python-install` so you can copy those steps when reproducing test failures.

## Packaging tips

- `python/templates/pyproject.toml.in` drives packaging in `build/python`.
- `make python-install` runs `pip install .` from `build/python`, which installs the generated extension module and package files from `build/python/gtdynamics`.
- If you need to publish a wheel manually, the wheel produced by `pip` already encodes the GTDynamics version reported by CMake.

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

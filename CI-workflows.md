# CI Workflows

The repository uses a single GitHub Actions workflow that runs on every pull request:

- `.github/workflows/common-ci.yml`

This workflow builds GTSAM from source and then builds/tests GTDynamics (both C++ and Python) across Ubuntu and macOS. It intentionally avoids relying on a prebuilt Docker image or a Homebrew tap for GTSAM so that a GTSAM fix is picked up as soon as it lands in the GTSAM repo's default branch.

## Common CI (`.github/workflows/common-ci.yml`)

### Platforms and toolchains
- **Ubuntu 24.04:** gcc-11 and clang-16
- **macOS 14:** Xcode 15.4

Each platform is tested in both Debug and Release configurations. The workflow uses Python 3 for the bindings build.

### How GTSAM is obtained (and what "version" that means)

The workflow clones https://github.com/borglab/gtsam.git and builds it from source with `-DGTSAM_BUILD_PYTHON=ON`.

That means the GTSAM version under test is always whatever commit is at the tip of the GTSAM repo's default branch at the time the workflow runs.

### How GTSAM is installed
- **Linux:** installed system-wide (via `sudo make install`), followed by `make python-install`.
- **macOS:** installed into a local prefix under the workspace (see the `GTSAM_INSTALL_DIR_MACOS` env var), using a Python virtual environment so the built bindings and `pip` share the same interpreter.

### What GTDynamics runs
- **Resource limits:** adds 6GB swap on Linux and caps build parallelism (Linux: `-j4`, macOS: `-j2`) to reduce OOM failures on GitHub-hosted runners.
- **Test deps:** installs `CppUnitLite` (required for `make check` C++ test targets)
- **C++ build:** `cmake -DGTDYNAMICS_BUILD_PYTHON=ON ...` then `make`
- **C++ tests:** `make check` (macOS sets `DYLD_LIBRARY_PATH` to find the locally installed GTSAM libraries)
- **Python install/tests:** `make python-install` and `make python-test`

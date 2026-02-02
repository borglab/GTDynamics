# CI Workflows

GTDynamics has two GitHub Actions workflows that run on every pull request:

- `.github/workflows/linux-ci.yml`
- `.github/workflows/macos-ci.yml`

They are intentionally kept as similar as possible: both build **GTSAM from source** first, then build/test **GTDynamics** against that installed GTSAM (C++ always; Python in Release only). This avoids relying on a prebuilt GTSAM package, so CI picks up fixes from GTSAM's default branch as soon as they land.

## Platforms and toolchains

- **Ubuntu 24.04:** gcc-14 and clang-16
- **macOS 15:** Xcode 16.4

Both platforms test **Debug** and **Release**.

## Build / test matrix behavior

- **Release:**
  - Builds GTSAM with Python wrappers (`GTSAM_BUILD_PYTHON=ON`)
  - Builds GTDynamics with Python wrappers (`GTDYNAMICS_BUILD_PYTHON=ON`)
  - Runs **C++ tests** (`make check`) and **Python tests** (`make python-test`)
- **Debug:**
  - Builds GTSAM without Python wrappers
  - Builds GTDynamics without Python wrappers
  - Runs **C++ tests only**

## Python environment (same on macOS and Linux)

In Release builds, both workflows create a Python virtual environment (`venv`) and pass its interpreter to CMake via `PYTHON_EXECUTABLE`. This ensures:

- The locally built `gtsam` Python package (from the GTSAM build) is installed into the same environment used to install/test `gtdynamics`.
- CI does not accidentally try to fetch `gtsam` from PyPI during `pip install .`.

## CPU flags / Eigen alignment consistency

Eigen's alignment requirements can change when AVX is enabled (e.g., via `-march=native`). CI keeps GTSAM and GTDynamics consistent in two ways:

- Release builds enable `-march=native` for clang on Linux; it is disabled for gcc on Linux.
- GTDynamics' CMake configuration matches `GTSAM_BUILD_WITH_MARCH_NATIVE` to the installed `gtsam` target's exported compile options, so the downstream build stays consistent even if the workflow does not explicitly set the flag for GTDynamics.

## Runtime library paths

Because GTSAM is installed under a workspace prefix in CI (not a system default), tests need appropriate dynamic loader search paths:

- **macOS:** `DYLD_LIBRARY_PATH` is set to `${GTSAM_INSTALL_DIR}/lib` during C++ and Python tests.
- **Linux:** `LD_LIBRARY_PATH` is set to `${GTSAM_INSTALL_DIR}/lib` for Python tests, and for gcc C++ tests as needed.

For local "build from source like CI" instructions, see the top-level `README.md` (section at the end).

#!/usr/bin/env bash
# Build wheels using cibuildwheel.
# Configuration is in pyproject.toml under [tool.cibuildwheel]

set -e -x

# Install cibuildwheel if not already installed
python3 -m pip install cibuildwheel

# Build the wheels
python3 -m cibuildwheel --output-dir wheelhouse

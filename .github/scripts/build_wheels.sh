#!/usr/bin/env bash

# This script builds the wheels using cibuildwheel.

set -e -x

# Install cibuildwheel if not already installed
python3 -m pip install cibuildwheel

# Build the wheels
python3 -m cibuildwheel --output-dir wheelhouse

#!/bin/bash
# This script runs once per platform in cibuildwheel's before-all hook.
# It installs all non-Python-dependent system dependencies.

set -e
set -x

NUM_CORES=$(sysctl -n hw.logicalcpu)

# Common installation prefix for all dependencies
export INSTALL_PREFIX="$HOME/opt/gtdynamics-deps"
mkdir -p ${INSTALL_PREFIX}

# If MACOSX_DEPLOYMENT_TARGET is not explicitly set, default to the host OS major version.
# Homebrew bottles are built against the runner's OS, so the deployment target must match.
if [[ -z "${MACOSX_DEPLOYMENT_TARGET}" ]]; then
    export MACOSX_DEPLOYMENT_TARGET="$(sw_vers -productVersion | cut -d. -f1).0"
fi

# Install Base System Dependencies via Homebrew
echo "Installing base system dependencies..."
brew install cmake wget git boost

# Install SDFormat via Homebrew
brew tap osrf/simulation
brew install sdformat15
brew --prefix urdfdom

# Clone GTSAM (don't build yet, depends on Python)
# GTDynamics requires gtsam-develop (not the 4.2 release tag)
echo "Cloning GTSAM source (develop branch)..."
git clone --depth 1 https://github.com/borglab/gtsam.git ${INSTALL_PREFIX}/gtsam_source

cat > ${INSTALL_PREFIX}/env.sh << EOF
export INSTALL_PREFIX="${INSTALL_PREFIX}"
export MACOSX_DEPLOYMENT_TARGET="${MACOSX_DEPLOYMENT_TARGET}"
export CMAKE_PREFIX_PATH="${BOOST_PREFIX}:\${CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="${BOOST_PREFIX}/lib:\${LD_LIBRARY_PATH}"
export DYLD_LIBRARY_PATH="${BOOST_PREFIX}/lib:\${DYLD_LIBRARY_PATH}"
export REPAIR_LIBRARY_PATH="${BOOST_PREFIX}/lib:\${REPAIR_LIBRARY_PATH}"
EOF

echo "before-all completed successfully!"

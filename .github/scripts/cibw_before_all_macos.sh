#!/bin/bash
# This script runs once per platform in cibuildwheel's before-all hook.
# It installs all non-Python-dependent system dependencies.

set -e
set -x

NUM_CORES=$(sysctl -n hw.logicalcpu)

# Common installation prefix for all dependencies
export INSTALL_PREFIX="$HOME/opt/gtdynamics-deps"
mkdir -p ${INSTALL_PREFIX}

# If MACOSX_DEPLOYMENT_TARGET is not explicitly set, default to the version of the host system.
if [[ -z "${MACOSX_DEPLOYMENT_TARGET}" ]]; then
    export MACOSX_DEPLOYMENT_TARGET="$(sw_vers -productVersion | cut -d '.' -f 1-2)"
fi

# Install Base System Dependencies via Homebrew
echo "Installing base system dependencies..."
brew install boost cmake wget git

# Install SDFormat via Homebrew
brew tap osrf/simulation
brew install sdformat15

# Clone GTSAM (don't build yet, depends on Python)
echo "Cloning GTSAM source..."
GTSAM_VERSION="4.2"
git clone --branch ${GTSAM_VERSION} --depth 1 https://github.com/borglab/gtsam.git ${INSTALL_PREFIX}/gtsam_source

# Write environment file for before-build scripts
BOOST_PREFIX=$(brew --prefix boost)s Python-dependent)

cat > ${INSTALL_PREFIX}/env.sh << EOF
export INSTALL_PREFIX="${INSTALL_PREFIX}"
export BOOST_ROOT="${BOOST_PREFIX}"
export BOOST_INCLUDEDIR="${BOOST_PREFIX}/include"
export BOOST_LIBRARYDIR="${BOOST_PREFIX}/lib"
export MACOSX_DEPLOYMENT_TARGET="${MACOSX_DEPLOYMENT_TARGET}"
export CMAKE_PREFIX_PATH="\${CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="${BOOST_PREFIX}/lib:\${LD_LIBRARY_PATH}"
export DYLD_LIBRARY_PATH="${BOOST_PREFIX}/lib:\${DYLD_LIBRARY_PATH}"
export REPAIR_LIBRARY_PATH="${BOOST_PREFIX}/lib:\${REPAIR_LIBRARY_PATH}"
EOF

echo "before-all completed successfully!"

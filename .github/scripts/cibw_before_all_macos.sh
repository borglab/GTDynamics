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
brew install cmake wget git

# Build Boost 1.87.0 from source (must match the version used by gtsam-develop on PyPI)
echo "Building Boost 1.87.0 from source..."
cd /tmp
wget https://archives.boost.io/release/1.87.0/source/boost_1_87_0.tar.gz --quiet
tar -xzf boost_1_87_0.tar.gz
cd boost_1_87_0

BOOST_PREFIX="${INSTALL_PREFIX}/boost"
./bootstrap.sh --prefix=${BOOST_PREFIX}
./b2 install --prefix=${BOOST_PREFIX} --with=all -d0 \
    cxxflags="-mmacosx-version-min=${MACOSX_DEPLOYMENT_TARGET}" \
    linkflags="-mmacosx-version-min=${MACOSX_DEPLOYMENT_TARGET}"
cd /tmp && rm -rf boost_1_87_0*

# Install SDFormat via Homebrew
brew tap osrf/simulation
brew install sdformat15

# Clone GTSAM (don't build yet, depends on Python)
# GTDynamics requires gtsam-develop (not the 4.2 release tag)
echo "Cloning GTSAM source (develop branch)..."
git clone --depth 1 https://github.com/borglab/gtsam.git ${INSTALL_PREFIX}/gtsam_source

# Write environment file for before-build scripts
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

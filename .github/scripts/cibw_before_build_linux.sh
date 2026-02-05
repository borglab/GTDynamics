#!/bin/bash
# This script runs once per Python version in cibuildwheel's before-build hook.
# It installs Python-dependent packages and builds GTSAM for the specific Python version.

set -e
set -x

PROJECT_DIR="$1"
INSTALL_PREFIX="/opt/gtdynamics-deps"
NUM_CORES=$(nproc)

# Source environment from before-all
source ${INSTALL_PREFIX}/env.sh

# Get the Python executable provided by cibuildwheel
PYTHON_EXE=$(which python)
PYTHON_VERSION=$($PYTHON_EXE -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")

echo "Building for Python ${PYTHON_VERSION} at ${PYTHON_EXE}"

# Install Python dependencies for GTSAM build and wheel build (no isolation)
echo "Installing Python dependencies..."
$PYTHON_EXE -m pip install --upgrade pip
$PYTHON_EXE -m pip install numpy pyparsing pybind11 scikit-build-core ninja "cmake>=3.26,<4"

# Define GTSAM paths for this Python version
GTSAM_SOURCE="${INSTALL_PREFIX}/gtsam_source"
GTSAM_BUILD="${INSTALL_PREFIX}/gtsam_build_py${PYTHON_VERSION}"
GTSAM_PREFIX="${INSTALL_PREFIX}/gtsam_py${PYTHON_VERSION}"


# Build and Install GTSAM for this Python version
echo "Building GTSAM for Python ${PYTHON_VERSION}..."

# Clean previous build if exists
rm -rf ${GTSAM_BUILD} ${GTSAM_PREFIX}
mkdir -p ${GTSAM_BUILD}

cd ${GTSAM_BUILD}
cmake ${GTSAM_SOURCE} \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${GTSAM_PREFIX} \
    -DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}" \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_UNSTABLE=ON \
    -DGTSAM_USE_QUATERNIONS=OFF \
    -DGTSAM_WITH_TBB=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_BUILD_PYTHON=ON \
    -DGTSAM_UNSTABLE_BUILD_PYTHON=ON \
    -DGTSAM_PYTHON_VERSION=${PYTHON_VERSION} \
    -DPYTHON_EXECUTABLE:FILEPATH=${PYTHON_EXE} \
    -DGTSAM_ALLOW_DEPRECATED_SINCE_V43=OFF \
    -DCMAKE_CXX_FLAGS="-faligned-new"

cmake --build . --config Release -j${NUM_CORES}
cmake --install .

# Install GTSAM Python package
echo "Installing GTSAM Python package..."
cd ${GTSAM_BUILD}/python
$PYTHON_EXE -m pip install .


# Update environment variables for GTDynamics build
# These exports will be visible to the pip wheel build that follows
export GTSAM_DIR="${GTSAM_PREFIX}/lib/cmake/GTSAM"
export CMAKE_PREFIX_PATH="${GTSAM_PREFIX}:${CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="${GTSAM_PREFIX}/lib:${LD_LIBRARY_PATH}"

# Also update ldconfig so libraries can be found
echo "${GTSAM_PREFIX}/lib" > /etc/ld.so.conf.d/gtsam.conf 2>/dev/null || true
ldconfig 2>/dev/null || true

# Create a symlink so we have a consistent path for the current Python version's GTSAM
rm -f ${INSTALL_PREFIX}/gtsam_current
ln -sf ${GTSAM_PREFIX} ${INSTALL_PREFIX}/gtsam_current

echo "============================================"
echo "GTSAM installed to: ${GTSAM_PREFIX}"
echo "GTSAM_DIR=${GTSAM_DIR}"
echo "CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}"
echo "LD_LIBRARY_PATH=${LD_LIBRARY_PATH}"
echo "============================================"

echo "before-build completed successfully for Python ${PYTHON_VERSION}!"
#!/bin/bash
# This script builds GTSAM and GTDynamics using a clean prefix-installation strategy.
# This prevents "Double Linking" of Boost and other dependencies.

set -e
set -x

PROJECT_DIR="$(cd "$1" && pwd)"
INSTALL_PREFIX="/opt/gtdynamics-deps"
NUM_CORES=$(nproc)

# Source environment from before-all (defines BOOST_ROOT, etc.)
source ${INSTALL_PREFIX}/env.sh

# Get the Python executable provided by cibuildwheel
PYTHON_EXE=$(which python)
PYTHON_VERSION=$($PYTHON_EXE -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")

echo "Building for Python ${PYTHON_VERSION} at ${PYTHON_EXE}"

# Install Python build-time dependencies
$PYTHON_EXE -m pip install --upgrade pip
$PYTHON_EXE -m pip install numpy pyparsing pybind11 hatchling ninja "cmake>=3.26,<4"

# Define Paths
GTSAM_SOURCE="${INSTALL_PREFIX}/gtsam_source"
GTSAM_BUILD="${INSTALL_PREFIX}/gtsam_build_py${PYTHON_VERSION}"
GTSAM_PREFIX="${INSTALL_PREFIX}/gtsam_py${PYTHON_VERSION}"

# 1. Build and INSTALL GTSAM to a clean prefix
echo "Building GTSAM..."
rm -rf ${GTSAM_BUILD} ${GTSAM_PREFIX}
mkdir -p ${GTSAM_BUILD}
cd ${GTSAM_BUILD}
cmake ${GTSAM_SOURCE} \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${GTSAM_PREFIX} \
    -DGTSAM_BUILD_PYTHON=ON \
    -DGTSAM_PYTHON_VERSION=${PYTHON_VERSION} \
    -DPYTHON_EXECUTABLE:FILEPATH=${PYTHON_EXE} \
    -DBOOST_ROOT=${BOOST_ROOT} \
    -DCMAKE_CXX_FLAGS="-faligned-new -Wno-error=free-nonheap-object" \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_INSTALL_LIBDIR=lib \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_UNSTABLE=ON

cmake --build . --config Release -j${NUM_CORES}
cmake --install .

# 2. Stage GTSAM Python package for bundling
echo "Staging GTSAM for bundling..."
GTSAM_PY_DST="${PROJECT_DIR}/python/gtsam"
rm -rf ${GTSAM_PY_DST}
mkdir -p ${GTSAM_PY_DST}

# Find where CMake installed the gtsam python package
GTSAM_PKG_SRC=$(find ${GTSAM_PREFIX} -name "gtsam" -type d | grep "site-packages/gtsam" | head -n 1)
if [ -z "$GTSAM_PKG_SRC" ]; then
    echo "ERROR: Could not find gtsam package in ${GTSAM_PREFIX}"
    exit 1
fi
cp -r "${GTSAM_PKG_SRC}/." "${GTSAM_PY_DST}/"
rm -rf ${GTSAM_PY_DST}/tests ${GTSAM_PY_DST}/examples ${GTSAM_PY_DST}/notebooks ${GTSAM_PY_DST}/__pycache__

# 3. Build and INSTALL GTDynamics to its own prefix
echo "Building GTDynamics..."
GTD_BUILD="${INSTALL_PREFIX}/gtd_build_py${PYTHON_VERSION}"
GTD_PREFIX="${INSTALL_PREFIX}/gtd_py${PYTHON_VERSION}"
rm -rf ${GTD_BUILD} ${GTD_PREFIX}
mkdir -p ${GTD_BUILD}
cd ${GTD_BUILD}
cmake ${PROJECT_DIR} \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${GTD_PREFIX} \
    -DGTSAM_DIR="${GTSAM_PREFIX}/lib/cmake/GTSAM" \
    -DGTDYNAMICS_BUILD_PYTHON=ON \
    -DBUILD_TESTING=OFF \
    -DPYTHON_EXECUTABLE:FILEPATH=${PYTHON_EXE} \
    -DBOOST_ROOT=${BOOST_ROOT} \
    -DCMAKE_CXX_FLAGS="-faligned-new" \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_INSTALL_LIBDIR=lib

cmake --build . --config Release --target gtdynamics_py -j${NUM_CORES}
cmake --install .

# 4. Stage GTDynamics extension for bundling
echo "Staging GTDynamics extension..."
# Grab ONLY the .so from the clean install prefix
find "${GTD_PREFIX}" -name "gtdynamics*.so" -exec cp -v {} "${PROJECT_DIR}/python/gtdynamics/" \;

# Finalize Environment for Auditwheel
rm -f ${INSTALL_PREFIX}/gtsam_current ${INSTALL_PREFIX}/gtd_current
ln -sf ${GTSAM_PREFIX} ${INSTALL_PREFIX}/gtsam_current
ln -sf ${GTD_PREFIX} ${INSTALL_PREFIX}/gtd_current

echo "before-build completed successfully!"
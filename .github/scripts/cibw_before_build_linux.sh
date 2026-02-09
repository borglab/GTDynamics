#!/bin/bash
# This script runs once per Python version in cibuildwheel's before-build hook.
# It installs Python-dependent packages and builds GTSAM/GTDynamics using prefix-installation.

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

# Install Python dependencies
echo "Installing Python dependencies..."
$PYTHON_EXE -m pip install --upgrade pip
$PYTHON_EXE -m pip install numpy pyparsing pybind11 hatchling ninja "cmake>=3.26,<4"

# Define Paths
GTSAM_SOURCE="${INSTALL_PREFIX}/gtsam_source"
GTSAM_BUILD="${INSTALL_PREFIX}/gtsam_build_py${PYTHON_VERSION}"
GTSAM_PREFIX="${INSTALL_PREFIX}/gtsam_py${PYTHON_VERSION}"

# 1. Build and INSTALL GTSAM to a clean prefix
echo "Building GTSAM for Python ${PYTHON_VERSION}..."
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
    -DBOOST_ROOT=${BOOST_ROOT} \
    -DCMAKE_CXX_FLAGS="-faligned-new -Wno-error=free-nonheap-object" \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_INSTALL_LIBDIR=lib

cmake --build . --config Release -j${NUM_CORES}
cmake --install .

# Install GTSAM into the build environment for GTDynamics compilation
echo "Installing GTSAM Python package to environment..."
cd ${GTSAM_BUILD}/python
$PYTHON_EXE -m pip install .

# 2. Copy GTSAM from the CLEAN install prefix (not the build tree)
echo "Staging GTSAM for bundling..."
GTSAM_PY_DST="${PROJECT_DIR}/python/gtsam"
rm -rf ${GTSAM_PY_DST}
mkdir -p ${GTSAM_PY_DST}

# Copy the Python package that was just installed to the prefix
cp -r ${GTSAM_PREFIX}/python/gtsam/* ${GTSAM_PY_DST}/

# Remove unneeded artifacts
rm -rf ${GTSAM_PY_DST}/tests ${GTSAM_PY_DST}/examples ${GTSAM_PY_DST}/notebooks ${GTSAM_PY_DST}/__pycache__

# 3. Build and INSTALL GTDynamics to its own prefix
echo "Building GTDynamics C++ extension..."
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
    -DGTDYNAMICS_BUILD_EXAMPLES=OFF \
    -DBUILD_TESTING=OFF \
    -DPython_EXECUTABLE:FILEPATH=${PYTHON_EXE} \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DBOOST_ROOT=${BOOST_ROOT} \
    -DCMAKE_CXX_FLAGS="-faligned-new" \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_INSTALL_LIBDIR=lib

cmake --build . --config Release --target gtdynamics_py -j${NUM_CORES}
cmake --install .

# 4. Copy GTDynamics extension from the CLEAN install prefix
echo "Staging GTDynamics extension for bundling..."
find ${GTD_PREFIX} -name "gtdynamics*.so" -exec cp {} ${PROJECT_DIR}/python/gtdynamics/ \;

# Setup symlinks and environment for auditwheel
rm -f ${INSTALL_PREFIX}/gtsam_current ${INSTALL_PREFIX}/gtd_current
ln -sf ${GTSAM_PREFIX} ${INSTALL_PREFIX}/gtsam_current
ln -sf ${GTD_PREFIX} ${INSTALL_PREFIX}/gtd_current

# Update ldconfig so the repair phase can find libraries
echo "${GTSAM_PREFIX}/lib" > /etc/ld.so.conf.d/gtsam.conf 2>/dev/null || true
echo "${GTD_PREFIX}/lib" > /etc/ld.so.conf.d/gtdynamics.conf 2>/dev/null || true
ldconfig 2>/dev/null || true

echo "============================================"
echo "GTSAM Prefix: ${GTSAM_PREFIX}"
echo "GTDynamics Prefix: ${GTD_PREFIX}"
echo "============================================"

echo "before-build completed successfully!"
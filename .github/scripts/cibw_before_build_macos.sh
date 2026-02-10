#!/bin/bash
# This script runs once per Python version in cibuildwheel's before-build hook.
# It installs Python-dependent packages and builds GTSAM for the specific Python version.

set -e
set -x

PROJECT_DIR="$(cd "$1" && pwd)"
INSTALL_PREFIX="$HOME/opt/gtdynamics-deps"
NUM_CORES=$(sysctl -n hw.logicalcpu)

# Source environment from before-all
source ${INSTALL_PREFIX}/env.sh

# Get the Python executable provided by cibuildwheel (full path)
# Use `python -c` so we get the exact interpreter executable path
PYTHON_EXE=$(python -c "import sys; print(sys.executable)")
PYTHON_VERSION=$($PYTHON_EXE -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")

echo "Building for Python ${PYTHON_VERSION} at ${PYTHON_EXE}"


# Install Python dependencies for GTSAM build and wheel build (no isolation)
echo "Installing Python dependencies..."
$PYTHON_EXE -m pip install --upgrade pip
$PYTHON_EXE -m pip install numpy pyparsing pybind11 hatchling ninja "cmake>=3.26,<4"


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
    -DGTSAM_BUILD_PYTHON=ON \
    -DGTSAM_PYTHON_VERSION=${PYTHON_VERSION} \
    -DPYTHON_EXECUTABLE:FILEPATH=${PYTHON_EXE} \
    -DGTSAM_ENABLE_BOOST_SERIALIZATION=OFF \
    -DGTSAM_USE_BOOST_FEATURES=OFF \
    -DCMAKE_CXX_FLAGS="-faligned-new -Wno-error=free-nonheap-object" \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_INSTALL_LIBDIR=lib \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_INSTALL_EXAMPLES=OFF \
    -DGTSAM_BUILD_UNSTABLE=ON

cmake --build . --config Release -j${NUM_CORES}
cmake --install .


# Stage GTSAM
echo "Staging GTSAM Python package for bundling..."
GTSAM_PY_SRC="${GTSAM_BUILD}/python/gtsam"
GTSAM_PY_DST="${PROJECT_DIR}/python/gtsam"
rm -rf ${GTSAM_PY_DST}
mkdir -p ${GTSAM_PY_DST}

cp ${GTSAM_PY_SRC}/__init__.py ${GTSAM_PY_DST}/
cp ${GTSAM_PY_SRC}/__init__.pyi ${GTSAM_PY_DST}/ 2>/dev/null || true
cp ${GTSAM_PY_SRC}/*.py ${GTSAM_PY_DST}/
find ${GTSAM_PY_SRC} -maxdepth 1 -name "gtsam*.so" -exec cp {} ${GTSAM_PY_DST}/ \;

for subdir in utils preamble specializations gtsam Data; do
    if [ -d "${GTSAM_PY_SRC}/${subdir}" ]; then
        cp -r ${GTSAM_PY_SRC}/${subdir} ${GTSAM_PY_DST}/
    fi
done

rm -rf ${GTSAM_PY_DST}/tests ${GTSAM_PY_DST}/examples ${GTSAM_PY_DST}/notebooks ${GTSAM_PY_DST}/__pycache__

# Add rpath to GTSAM libraries for macOS
for dylib in ${GTSAM_PREFIX}/lib/*.dylib; do
    if [ -f "$dylib" ]; then
        install_name_tool -add_rpath "@loader_path" "$dylib" 2>/dev/null || true
    fi
done


# Update environment variables for GTDynamics build
# These exports will be visible to the pip wheel build that follows
export GTSAM_DIR="${GTSAM_PREFIX}/lib/cmake/GTSAM"
export CMAKE_PREFIX_PATH="${GTSAM_PREFIX}:${CMAKE_PREFIX_PATH}"
export DYLD_LIBRARY_PATH="${GTSAM_PREFIX}/lib:${DYLD_LIBRARY_PATH}"

# Create a symlink so we have a consistent path for the current Python version's GTSAM
rm -f ${INSTALL_PREFIX}/gtsam_current
ln -sf ${GTSAM_PREFIX} ${INSTALL_PREFIX}/gtsam_current

# Patch gtwrap to use Development.Module instead of Development.
# macOS Python from cibuildwheel may not ship full libpython.
sed -i '' 's/Interpreter Development/Interpreter Development.Module/g' \
    ${GTSAM_PREFIX}/lib/cmake/gtwrap/GtwrapUtils.cmake


# Build GTDynamics C++ extension for this Python version
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
    -DPYTHON_EXECUTABLE:FILEPATH=${PYTHON_EXE} \
    -DGTDYNAMICS_ENABLE_BOOST_SERIALIZATION=OFF \
    -DCMAKE_CXX_FLAGS="-faligned-new" \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_INSTALL_LIBDIR=lib

cmake --build . --config Release --target gtdynamics_py -j${NUM_CORES}
cmake --install .

# Copy the built Python extension (.so) into the source tree so that
# hatchling (a pure-Python build backend) can package it into the wheel.
echo "Copying built extension to source tree..."
find ${GTD_BUILD}/python/gtdynamics -name "gtdynamics*.so" \
    -exec cp {} ${PROJECT_DIR}/python/gtdynamics/ \;

# Install GTDynamics shared library where delocate-wheel can find it
mkdir -p ${GTD_PREFIX}/lib
find ${GTD_BUILD} -maxdepth 2 -name "libgtdynamics*.dylib" \
    -exec cp {} ${GTD_PREFIX}/lib/ \;
rm -f ${INSTALL_PREFIX}/gtd_current
ln -sf ${GTD_PREFIX} ${INSTALL_PREFIX}/gtd_current

# Remove any duplicate gtsam dylibs from GTDynamics lib directory
find ${GTD_PREFIX}/lib -name "libgtsam*.dylib" -delete

export DYLD_LIBRARY_PATH="${GTD_PREFIX}/lib:${DYLD_LIBRARY_PATH}"

echo "before-build completed successfully for Python ${PYTHON_VERSION}!"
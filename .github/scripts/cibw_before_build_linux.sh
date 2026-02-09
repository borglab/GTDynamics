#!/bin/bash
# Refined Hybrid Build Script: Proven Copy Logic + Clean Prefix Strategy
set -e
set -x

PROJECT_DIR="$(cd "$1" && pwd)"
INSTALL_PREFIX="/opt/gtdynamics-deps"
NUM_CORES=$(nproc)

# 1. Setup Environment
source ${INSTALL_PREFIX}/env.sh
PYTHON_EXE=$(which python)
PYTHON_VERSION=$($PYTHON_EXE -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")

echo "Building for Python ${PYTHON_VERSION} at ${PYTHON_EXE}"

$PYTHON_EXE -m pip install --upgrade pip
$PYTHON_EXE -m pip install numpy pyparsing pybind11 hatchling ninja "cmake>=3.26,<4"

# Define Paths
GTSAM_SOURCE="${INSTALL_PREFIX}/gtsam_source"
GTSAM_BUILD="${INSTALL_PREFIX}/gtsam_build_py${PYTHON_VERSION}"
GTSAM_PREFIX="${INSTALL_PREFIX}/gtsam_py${PYTHON_VERSION}"

# 2. Build and INSTALL GTSAM to Clean Prefix
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

# 3. Stage GTSAM using your Working Copy Logic
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

# 4. Finalize Environment for GTDynamics
export GTSAM_DIR="${GTSAM_PREFIX}/lib/cmake/GTSAM"
export CMAKE_PREFIX_PATH="${GTSAM_PREFIX}:${CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="${GTSAM_PREFIX}/lib:${LD_LIBRARY_PATH}"

rm -f ${INSTALL_PREFIX}/gtsam_current
ln -sf ${GTSAM_PREFIX} ${INSTALL_PREFIX}/gtsam_current

# Patch gtwrap for manylinux compatibility
sed -i 's/Interpreter Development/Interpreter Development.Module/g' \
    ${GTSAM_PREFIX}/lib/cmake/gtwrap/GtwrapUtils.cmake

# 5. Build and INSTALL GTDynamics
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


# 6. Stage GTDynamics extension 
echo "Staging GTDynamics extension..."
GTD_PY_STAGING="${GTD_BUILD}/python/gtdynamics"

# Find the .so 
GTD_SO=$(find "${GTD_PY_STAGING}" -maxdepth 1 -name "gtdynamics*.so" -print -quit)

if [ -z "$GTD_SO" ]; then
    echo "ERROR: Could not find gtdynamics .so in ${GTD_PY_STAGING}"
    exit 1
fi

# Copy to source tree for Hatchling to pick up
cp -v "$GTD_SO" "${PROJECT_DIR}/python/gtdynamics/gtdynamics.so"

# Finalize for Auditwheel
export LD_LIBRARY_PATH="${GTD_PREFIX}/lib:${GTSAM_PREFIX}/lib:${LD_LIBRARY_PATH}"

echo "before-build completed successfully!"
#!/bin/bash
# This script runs once per platform in cibuildwheel's before-all hook.
# It installs all non-Python-dependent system dependencies.

set -e
set -x

ARCH=$(uname -m)
NUM_CORES=$(nproc)

# Common installation prefix for all dependencies
export INSTALL_PREFIX="/opt/gtdynamics-deps"
mkdir -p ${INSTALL_PREFIX}


# Install Base System Dependencies (manylinux_2_34 uses dnf)
echo "Installing base system dependencies..."
dnf install -y \
    wget curl git \
    tinyxml2-devel \
    ruby

echo "CMake version:"
cmake --version


# Build Boost 1.87.0 from source (must match the version used by gtsam-develop on PyPI)
echo "Building Boost 1.87.0 from source..."
cd /tmp
wget https://archives.boost.io/release/1.87.0/source/boost_1_87_0.tar.gz --quiet
tar -xzf boost_1_87_0.tar.gz
cd boost_1_87_0

BOOST_PREFIX="${INSTALL_PREFIX}/boost"
./bootstrap.sh --prefix=${BOOST_PREFIX}
./b2 install --prefix=${BOOST_PREFIX} --with=all -d0
cd /tmp && rm -rf boost_1_87_0*

export BOOST_ROOT="${BOOST_PREFIX}"
export BOOST_INCLUDEDIR="${BOOST_PREFIX}/include"
export BOOST_LIBRARYDIR="${BOOST_PREFIX}/lib"


# Install Gazebo dependencies for SDFormat
echo "Installing Gazebo dependencies..."
cd /tmp

# Install gz-cmake4
GZ_CMAKE_VERSION="4.1.0"
wget https://github.com/gazebosim/gz-cmake/archive/refs/tags/gz-cmake4_${GZ_CMAKE_VERSION}.tar.gz --quiet
tar -xzf gz-cmake4_${GZ_CMAKE_VERSION}.tar.gz
cd gz-cmake-gz-cmake4_${GZ_CMAKE_VERSION}
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${INSTALL_PREFIX}/gz-cmake4 -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build . --config Release -j${NUM_CORES}
cmake --install .
cd /tmp && rm -rf gz-cmake*

# Install gz-utils
GZ_UTILS_VERSION="3.0.0"
wget https://github.com/gazebosim/gz-utils/archive/refs/tags/gz-utils3_${GZ_UTILS_VERSION}.tar.gz --quiet
tar -xzf gz-utils3_${GZ_UTILS_VERSION}.tar.gz
cd gz-utils-gz-utils3_${GZ_UTILS_VERSION}
mkdir -p build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${INSTALL_PREFIX}/gz-utils \
    -DCMAKE_PREFIX_PATH="${INSTALL_PREFIX}/gz-cmake4" \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build . --config Release -j${NUM_CORES}
cmake --install .
cd /tmp && rm -rf gz-utils*

# Install gz-math
GZ_MATH_VERSION="8.0.0"
wget https://github.com/gazebosim/gz-math/archive/refs/tags/gz-math8_${GZ_MATH_VERSION}.tar.gz --quiet
tar -xzf gz-math8_${GZ_MATH_VERSION}.tar.gz
cd gz-math-gz-math8_${GZ_MATH_VERSION}
mkdir -p build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${INSTALL_PREFIX}/gz-math \
    -DCMAKE_PREFIX_PATH="${INSTALL_PREFIX}/gz-cmake4;${INSTALL_PREFIX}/gz-utils" \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build . --config Release -j${NUM_CORES}
cmake --install .
cd /tmp && rm -rf gz-math*

# Install SDFormat
echo "Installing SDFormat..."
SDFORMAT_VERSION="15.1.1"
wget https://github.com/gazebosim/sdformat/archive/refs/tags/sdformat15_${SDFORMAT_VERSION}.tar.gz --quiet
tar -xzf sdformat15_${SDFORMAT_VERSION}.tar.gz
cd sdformat-sdformat15_${SDFORMAT_VERSION}
mkdir -p build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${INSTALL_PREFIX}/sdformat \
    -DCMAKE_PREFIX_PATH="${INSTALL_PREFIX}/gz-cmake4;${INSTALL_PREFIX}/gz-utils;${INSTALL_PREFIX}/gz-math" \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DSKIP_PYBIND11=ON \
    -DBUILD_TESTING=OFF
cmake --build . --config Release -j${NUM_CORES}
cmake --install .
cd /tmp && rm -rf sdformat*


# Clone GTSAM (don't build yet, depends on Python)
# GTDynamics requires gtsam-develop (not the 4.2 release tag)
echo "Cloning GTSAM source (develop branch)..."
git clone --depth 1 https://github.com/borglab/gtsam.git ${INSTALL_PREFIX}/gtsam_source

# Write environment file for before-build scripts
# gtsam_current symlink will be created by before-build after GTSAM is built
cat > ${INSTALL_PREFIX}/env.sh << EOF
export INSTALL_PREFIX="${INSTALL_PREFIX}"
export BOOST_ROOT="${BOOST_PREFIX}"
export BOOST_INCLUDEDIR="${BOOST_PREFIX}/include"
export BOOST_LIBRARYDIR="${BOOST_PREFIX}/lib"
export CMAKE_PREFIX_PATH="${INSTALL_PREFIX}/gtsam_current:${INSTALL_PREFIX}/gz-cmake4:${INSTALL_PREFIX}/gz-utils:${INSTALL_PREFIX}/gz-math:${INSTALL_PREFIX}/sdformat:\${CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="${BOOST_PREFIX}/lib:${INSTALL_PREFIX}/gtsam_current/lib:${INSTALL_PREFIX}/sdformat/lib:${INSTALL_PREFIX}/gz-utils/lib:${INSTALL_PREFIX}/gz-math/lib:\${LD_LIBRARY_PATH}"
EOF

echo "before-all completed successfully!"
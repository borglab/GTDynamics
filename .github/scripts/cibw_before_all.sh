#!/bin/bash
set -e -x

# This script is run before building the wheels with cibuildwheel.
# It installs all the dependencies needed to build GTDynamics and its Python wrapper.

PYTHON_VERSION=$1
PROJECT_DIR=$2

echo "Building dependencies for Python ${PYTHON_VERSION}"
echo "Project directory: ${PROJECT_DIR}"

# Detect OS
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    OS="linux"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    OS="macos"
else
    echo "Unsupported OS: $OSTYPE"
    exit 1
fi

# Install system dependencies based on OS
if [ "$OS" == "linux" ]; then
    echo "Installing system dependencies on Linux..."
    yum install -y wget tar gzip make gcc-c++ which
    
    # Install CMake
    CMAKE_VERSION=3.28.1
    wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-$(uname -m).tar.gz
    tar -xzf cmake-${CMAKE_VERSION}-linux-$(uname -m).tar.gz
    export PATH=$(pwd)/cmake-${CMAKE_VERSION}-linux-$(uname -m)/bin:$PATH
    cmake --version

elif [ "$OS" == "macos" ]; then
    echo "Installing system dependencies on macOS..."
    # CMake and ninja should already be installed on the host
    which cmake
    which ninja
fi

# Build and install Boost
echo "Building Boost..."
BOOST_VERSION=1.85.0
BOOST_VERSION_UNDERSCORE=${BOOST_VERSION//./_}
cd /tmp
wget https://boostorg.jfrog.io/artifactory/main/release/${BOOST_VERSION}/source/boost_${BOOST_VERSION_UNDERSCORE}.tar.gz
tar -xzf boost_${BOOST_VERSION_UNDERSCORE}.tar.gz
cd boost_${BOOST_VERSION_UNDERSCORE}
./bootstrap.sh --prefix=/usr/local --with-libraries=serialization,filesystem,thread,system,atomic,date_time,timer,chrono,program_options,regex
./b2 install -j$(nproc 2>/dev/null || sysctl -n hw.ncpu) || ./b2 install -j4
cd ..
rm -rf boost_${BOOST_VERSION_UNDERSCORE}*

# Build and install Eigen
echo "Building Eigen..."
cd /tmp
EIGEN_VERSION=3.4.0
wget https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/eigen-${EIGEN_VERSION}.tar.gz
tar -xzf eigen-${EIGEN_VERSION}.tar.gz
cd eigen-${EIGEN_VERSION}
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make install -j$(nproc 2>/dev/null || sysctl -n hw.ncpu) || make install -j4
cd ../..
rm -rf eigen-${EIGEN_VERSION}*

# Build and install GTSAM
echo "Building GTSAM..."
cd /tmp
GTSAM_VERSION=4.2.0
wget https://github.com/borglab/gtsam/archive/refs/tags/${GTSAM_VERSION}.tar.gz -O gtsam-${GTSAM_VERSION}.tar.gz
tar -xzf gtsam-${GTSAM_VERSION}.tar.gz
cd gtsam-${GTSAM_VERSION}
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_INSTALL_PREFIX=/usr/local \
         -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_UNSTABLE=OFF \
         -DGTSAM_USE_SYSTEM_EIGEN=ON \
         -DGTSAM_BUILD_PYTHON=OFF
make install -j$(nproc 2>/dev/null || sysctl -n hw.ncpu) || make install -j4
cd ../..
rm -rf gtsam-${GTSAM_VERSION}*

# Build and install SDFormat
echo "Building SDFormat..."
cd /tmp
SDFORMAT_VERSION=15.1.1
wget https://github.com/gazebosim/sdformat/archive/refs/tags/sdformat15_${SDFORMAT_VERSION}.tar.gz
tar -xzf sdformat15_${SDFORMAT_VERSION}.tar.gz
cd sdformat-sdformat15_${SDFORMAT_VERSION}
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_INSTALL_PREFIX=/usr/local \
         -DBUILD_TESTING=OFF \
         -DSKIP_PYBIND11=ON
make install -j$(nproc 2>/dev/null || sysctl -n hw.ncpu) || make install -j4
cd ../..
rm -rf sdformat-sdformat15_${SDFORMAT_VERSION}*

# Now configure GTDynamics to generate setup.py
echo "Configuring GTDynamics with Python support..."
cd "${PROJECT_DIR}"

# Create build directory if it doesn't exist
mkdir -p build
cd build

# Configure GTDynamics with Python wrapper enabled
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH=/usr/local \
    -DGTSAM_DIR=/usr/local/lib/cmake/GTSAM \
    -DGTDYNAMICS_BUILD_PYTHON=ON \
    -DGTDYNAMICS_PYTHON_VERSION=${PYTHON_VERSION} \
    -DBUILD_TESTING=OFF

# Verify setup.py was created
if [ ! -f "python/setup.py" ]; then
    echo "ERROR: setup.py was not generated!"
    exit 1
fi

echo "setup.py successfully generated at build/python/setup.py"
ls -la python/

# Set environment variables for the wheel build
export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH
export GTSAM_DIR=/usr/local/lib/cmake/GTSAM
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
export DYLD_LIBRARY_PATH=/usr/local/lib:${DYLD_LIBRARY_PATH:-}

echo "Dependencies installation complete!"
echo "CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH"
echo "GTSAM_DIR=$GTSAM_DIR"
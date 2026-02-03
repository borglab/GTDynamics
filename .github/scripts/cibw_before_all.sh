#!/usr/bin/env bash

# This script is called by cibuildwheel before building the wheels.
# It installs all dependencies needed to build GTDynamics wheels.

set -e -x

PYTHON_VERSION=$1
PROJECT_DIR=$2

echo "Python version: $PYTHON_VERSION"
echo "Project directory: $PROJECT_DIR"

# Detect OS
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    OS="linux"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    OS="macos"
else
    echo "Unsupported OS: $OSTYPE"
    exit 1
fi

echo "Operating system: $OS"

# Function to install dependencies on Linux (manylinux container)
install_linux_dependencies() {
    echo "Installing Linux dependencies..."
    
    # Install basic tools
    yum install -y wget tar gzip make gcc-c++

    # Install CMake 3.20+
    CMAKE_VERSION=3.28.1
    echo "Installing CMake ${CMAKE_VERSION}..."
    wget -q https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-$(uname -m).tar.gz
    tar -xzf cmake-${CMAKE_VERSION}-linux-$(uname -m).tar.gz
    cp -r cmake-${CMAKE_VERSION}-linux-$(uname -m)/* /usr/local/
    rm -rf cmake-${CMAKE_VERSION}-linux-$(uname -m)*
    cmake --version

    # Install Boost from source
    BOOST_VERSION=1.84.0
    BOOST_VERSION_UNDERSCORE=${BOOST_VERSION//./_}
    
    echo "Installing Boost ${BOOST_VERSION}..."
    wget -q https://boostorg.jfrog.io/artifactory/main/release/${BOOST_VERSION}/source/boost_${BOOST_VERSION_UNDERSCORE}.tar.gz
    tar -xzf boost_${BOOST_VERSION_UNDERSCORE}.tar.gz
    cd boost_${BOOST_VERSION_UNDERSCORE}
    
    ./bootstrap.sh --prefix=/usr/local --with-libraries=serialization,filesystem,thread,program_options,date_time,timer,chrono,regex,system
    ./b2 -j$(nproc) install
    
    export BOOST_ROOT=/usr/local
    export BOOST_LIBRARYDIR=/usr/local/lib
    
    cd ..
    rm -rf boost_${BOOST_VERSION_UNDERSCORE}*

    # Install TinyXML2 (required for SDFormat)
    echo "Installing TinyXML2..."
    TINYXML2_VERSION=10.0.0
    wget -q https://github.com/leethomason/tinyxml2/archive/refs/tags/${TINYXML2_VERSION}.tar.gz
    tar -xzf ${TINYXML2_VERSION}.tar.gz
    cd tinyxml2-${TINYXML2_VERSION}
    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
    make -j$(nproc) install
    cd ../..
    rm -rf tinyxml2-${TINYXML2_VERSION}*

    # Install SDFormat from source
    echo "Installing SDFormat..."
    SDFORMAT_VERSION=15.1.1
    wget -q https://github.com/gazebosim/sdformat/archive/refs/tags/sdformat${SDFORMAT_VERSION}.tar.gz -O sdformat.tar.gz
    tar -xzf sdformat.tar.gz
    cd sdformat-sdformat${SDFORMAT_VERSION}
    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release \
             -DCMAKE_INSTALL_PREFIX=/usr/local \
             -DBUILD_TESTING=OFF \
             -DSKIP_PYBIND11=ON
    make -j$(nproc) install
    cd ../..
    rm -rf sdformat-sdformat${SDFORMAT_VERSION}* sdformat.tar.gz
}

# Function to install dependencies on macOS
install_macos_dependencies() {
    echo "Installing macOS dependencies..."
    
    # Install Boost from source for ABI compatibility
    BOOST_VERSION=1.84.0
    BOOST_VERSION_UNDERSCORE=${BOOST_VERSION//./_}
    
    echo "Installing Boost ${BOOST_VERSION}..."
    curl -L -o boost.tar.gz https://boostorg.jfrog.io/artifactory/main/release/${BOOST_VERSION}/source/boost_${BOOST_VERSION_UNDERSCORE}.tar.gz
    tar -xzf boost.tar.gz
    cd boost_${BOOST_VERSION_UNDERSCORE}
    
    ./bootstrap.sh --prefix=/usr/local --with-libraries=serialization,filesystem,thread,program_options,date_time,timer,chrono,regex,system
    ./b2 -j$(sysctl -n hw.ncpu) install
    
    export BOOST_ROOT=/usr/local
    export BOOST_LIBRARYDIR=/usr/local/lib
    
    cd ..
    rm -rf boost_${BOOST_VERSION_UNDERSCORE}* boost.tar.gz
    
    # Install SDFormat via Homebrew (simpler on macOS)
    brew tap osrf/simulation
    brew install sdformat15
}

# Install GTSAM from source
install_gtsam() {
    echo "Installing GTSAM..."
    
    GTSAM_VERSION=4.2.0
    
    if [[ "$OS" == "linux" ]]; then
        wget -q https://github.com/borglab/gtsam/archive/refs/tags/${GTSAM_VERSION}.tar.gz -O gtsam.tar.gz
    else
        curl -L -o gtsam.tar.gz https://github.com/borglab/gtsam/archive/refs/tags/${GTSAM_VERSION}.tar.gz
    fi
    
    tar -xzf gtsam.tar.gz
    cd gtsam-${GTSAM_VERSION}
    mkdir build && cd build
    
    cmake .. -DCMAKE_BUILD_TYPE=Release \
             -DCMAKE_INSTALL_PREFIX=/usr/local \
             -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
             -DGTSAM_BUILD_TESTS=OFF \
             -DGTSAM_WITH_TBB=OFF \
             -DGTSAM_BUILD_PYTHON=OFF
    
    if [[ "$OS" == "linux" ]]; then
        make -j$(nproc) install
    else
        make -j$(sysctl -n hw.ncpu) install
    fi
    
    cd ../..
    rm -rf gtsam-${GTSAM_VERSION}* gtsam.tar.gz
}

# Main installation logic
if [[ "$OS" == "linux" ]]; then
    install_linux_dependencies
else
    install_macos_dependencies
fi

install_gtsam

# Set REPAIR_LIBRARY_PATH for macOS wheel repair
if [[ "$OS" == "macos" ]]; then
    export REPAIR_LIBRARY_PATH="${BOOST_LIBRARYDIR}:/usr/local/lib"
    echo "REPAIR_LIBRARY_PATH=${REPAIR_LIBRARY_PATH}" >> "$GITHUB_ENV"
fi

echo "Dependencies installation complete!"

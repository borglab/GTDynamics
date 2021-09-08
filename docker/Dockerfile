#  GTSAM Ubuntu image with Python wrapper support.

# Get the base Ubuntu/GTSAM (with wrapper) image from Docker Hub
FROM borglab/ubuntu-gtsam-python:bionic


# Make sure we are not in GTSAM dir
WORKDIR /usr/src/

# Make sure we have latest packages
RUN apt-get -y update

# Install basic dependencies for sdfformat
RUN apt-get install -y ruby-dev build-essential libboost-all-dev cmake pkg-config wget lsb-release

# Get Gazebo packages
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN apt-get update
RUN apt-get install -y libtinyxml2-dev liburdfdom-dev libignition-cmake2-dev libignition-tools-dev libignition-math6-dev python3-psutil

# Get sdfformat package
ENV GTD_SDFormat_VERSION="10.5.0"
RUN wget http://osrf-distributions.s3.amazonaws.com/sdformat/releases/sdformat-${GTD_SDFormat_VERSION}.tar.bz2
RUN tar -xvjf sdformat-${GTD_SDFormat_VERSION}.tar.bz2
RUN rm sdformat-${GTD_SDFormat_VERSION}.tar.bz2

# Change to build directory. Will be created automatically.
WORKDIR sdformat-${GTD_SDFormat_VERSION}/build

# Run cmake and make
RUN cmake ..
RUN make -j4 install

# Avoid ascii errors when reading files in Python
RUN apt-get install -y locales && locale-gen en_US.UTF-8
ENV LANG='en_US.UTF-8' LANGUAGE='en_US:en' LC_ALL='en_US.UTF-8'
# Update setuptools to avoid python wrapper install warnings
RUN pip3 install -U setuptools

# Clone GTDynamics (develop branch)
WORKDIR /usr/src/
RUN git clone https://github.com/borglab/GTDynamics.git

# Change to build directory. Will be created automatically.
WORKDIR /usr/src/GTDynamics/build

# Run cmake
RUN cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTDYNAMICS_BUILD_PYTHON=ON \
    -DWRAP_PYTHON_VERSION=3.6.9 \
    ..

# Build and install
RUN make -j4 install
RUN make python-install
RUN make clean

# Run bash
CMD 'bash'

#  GTSAM Ubuntu image with Python wrapper support.

# Get the base Ubuntu/GTSAM (with wrapper) image from Docker Hub
FROM borglab/ubuntu-gtsam-python:bionic

# Install basic dependencies for sdfformat
RUN apt-get -y install libtinyxml2-dev libxml2-utils pkg-config ruby-dev ruby wget

# sdformat requires libignition-math
RUN apt-get -y install libignition-math4-dev

# Download specific version of SDFormat
WORKDIR /usr/src/
RUN wget http://osrf-distributions.s3.amazonaws.com/sdformat/releases/sdformat-10.5.0.tar.bz2
RUN tar -xvjf sdformat-10.5.0.tar.bz2

# Change to build directory. Will be created automatically.
WORKDIR sdformat-10.5.0/build

# Run cmake and make
RUN cmake ..
RUN make -j4 install

# Clone GTDynamics (develop branch)
WORKDIR /usr/src/
RUN git clone https://github.com/borglab/GTDynamics.git

# Change to build directory. Will be created automatically.
WORKDIR /usr/src/GTDynamics/build

# Run cmake
RUN cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_BUILD_PYTHON=ON \
    -DGTSAM_PYTHON_VERSION=3\
    ..

# Build and install
RUN make -j4 install
RUN make python-install
RUN make clean

# Run bash
CMD 'bash'

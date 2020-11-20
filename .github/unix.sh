#!/bin/bash

##########################################################
# Build and test GTDynamics for *nix based systems.
# Specifically Linux and macOS.
##########################################################

# common tasks before either build or test
function configure()
{
  set -e   # Make sure any error makes the script to return an error code
  set -x   # echo

  SOURCE_DIR=$GITHUB_WORKSPACE
  BUILD_DIR=$GITHUB_WORKSPACE/build

  #env
  rm -fr $BUILD_DIR || true
  mkdir $BUILD_DIR && cd $BUILD_DIR

  if [ ! -z "$GCC_VERSION" ]; then
    export CC=gcc-$GCC_VERSION
    export CXX=g++-$GCC_VERSION
  fi

  cmake $SOURCE_DIR 
}


# common tasks after either build or test
function finish ()
{
  # Print ccache stats
  [ -x "$(command -v ccache)" ] && ccache -s

  cd $SOURCE_DIR
}

# compile the code with the intent of populating the cache
function build ()
{
  configure
  make -j2
  finish
}

# run the tests
function test ()
{
  configure
  # Actual build:
  make -j2 check
  finish
}

# select between build or test
case $1 in
  -b)
    build
    ;;
  -t)
    test
    ;;
esac
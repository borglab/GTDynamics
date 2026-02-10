#!/bin/bash
# This script runs once per platform in cibuildwheel's before-all hook.
# It installs all non-Python-dependent system dependencies.

set -e
set -x

NUM_CORES=$(sysctl -n hw.logicalcpu)

# Common installation prefix for all dependencies
export INSTALL_PREFIX="$HOME/opt/gtdynamics-deps"
mkdir -p ${INSTALL_PREFIX}

# If MACOSX_DEPLOYMENT_TARGET is not explicitly set, default to the host OS major version.
# Homebrew bottles are built against the runner's OS, so the deployment target must match.
if [[ -z "${MACOSX_DEPLOYMENT_TARGET}" ]]; then
    export MACOSX_DEPLOYMENT_TARGET="$(sw_vers -productVersion | cut -d. -f1).0"
fi

# Install Base System Dependencies via Homebrew
echo "Installing base system dependencies..."
brew install cmake wget git

# Install SDFormat via Homebrew
brew tap osrf/simulation
brew install urdfdom  # Install dependency explicitly first
brew upgrade urdfdom  # Force it to the latest version (5.1+)
brew install sdformat15
brew --prefix urdfdom

# Create symlinks for urdfdom 5.1
URDF_ROOT=$(brew --prefix urdfdom)
for lib in sensor model world; do
    if [ ! -f "$URDF_ROOT/lib/liburdfdom_${lib}.5.1.dylib" ]; then
        echo "Creating safety symlink for liburdfdom_${lib}.5.1.dylib"
        ln -s "$URDF_ROOT/lib/liburdfdom_${lib}.dylib" "$URDF_ROOT/lib/liburdfdom_${lib}.5.1.dylib" || true
    fi
done


# Clone GTSAM (don't build yet, depends on Python)
# GTDynamics requires gtsam-develop (not the 4.2 release tag)
echo "Cloning GTSAM source (develop branch)..."
git clone --depth 1 https://github.com/borglab/gtsam.git ${INSTALL_PREFIX}/gtsam_source

cat > ${INSTALL_PREFIX}/env.sh << EOF
export INSTALL_PREFIX="${INSTALL_PREFIX}"
export MACOSX_DEPLOYMENT_TARGET="${MACOSX_DEPLOYMENT_TARGET}"
EOF

echo "before-all completed successfully!"

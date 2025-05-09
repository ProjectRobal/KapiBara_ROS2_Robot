#!/bin/bash

source /opt/ros/humble/setup.bash

cd /app/src/workspace

# sudo apt-get update

# rosdep update

rosdep install --from-paths /app/src/workspace/src --ignore-src -r -y -q

colcon build --symlink-install --build-base $BUILD_LOCATION --install-base $INSTALL_LOCATION --packages-ignore $1
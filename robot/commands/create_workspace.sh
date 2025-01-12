#!/bin/bash

source /opt/ros/humble/setup.bash

cd /app/src/workspace
colcon build --symlink-install --build-base $BUILD_LOCATION --install-base $INSTALL_LOCATION
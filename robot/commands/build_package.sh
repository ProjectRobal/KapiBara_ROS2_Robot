#!/bin/bash

source /opt/ros/iron/setup.bash

cd /app/src/workspace

rosdep install --from-paths /app/src/workspace/src --ignore-src -r -y -q

colcon build --symlink-install --packages-select $1
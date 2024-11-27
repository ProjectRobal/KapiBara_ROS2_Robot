#!/bin/bash

udevadm control --reload 

source /opt/ros/humble/setup.bash
source /app/src/workspace/install/setup.bash

cd /app/src

# sudo apt-get update

# rosdep update

rosdep install --from-paths /app/src/workspace/src --ignore-src  -r -y -q

find /app/src/workspace/src -name "dependencies.txt" -exec pip install -r {} \;

ros2 launch ./launch/launch.py
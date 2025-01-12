#!/bin/bash

udevadm control --reload 

source /opt/ros/humble/setup.bash
source /app/src/workspace/sim_install/setup.bash


cd /app/src

# sudo apt-get update

# rosdep update

pip3 install --upgrade pip 

# rosdep install --from-paths /app/src/workspace/src --ignore-src  -r -y -q

# find /app/src/workspace/src -name "dependencies.txt" -exec pip3 install -r {} \;

ros2 launch ./launch/launch.sim.py
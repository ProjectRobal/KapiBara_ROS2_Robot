#!/bin/bash

source /opt/ros/iron/setup.bash
source /app/src/workspace/install/setup.bash

cd /app/src

sudo apt-get update

rosdep update

rosdep install --from-paths /app/src/workspace/src --ignore-src -r -y -q

ros2 launch ./launch/launch.py

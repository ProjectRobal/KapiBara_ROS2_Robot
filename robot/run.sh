#!/bin/bash

source /opt/ros/humble/setup.bash
source /app/src/workspace/install/setup.bash

cd /app/src

exec ros2 launch ./launch/launch.py
#!/bin/bash

sudo apt update

sudo xargs apt -y install < /app/dep/packages.txt

sudo xargs apt -y install < /app/dep/gazebo.txt

rosdep update

pip3 install --upgrade pip 

rosdep install --from-paths /app/src/workspace/src --ignore-src  -r -y -q

find /app/src/workspace/src -name "dependencies.txt" -exec pip3 install -r {} \;

sudo apt update -y

source /opt/ros/humble/setup.bash
source /app/src/workspace/sim_install/setup.bash

cd /app/src

exec ros2 launch ./launch/launch.sim.py sigterm_timeout:=1800
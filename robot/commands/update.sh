#!/bin/bash

sudo apt update

sudo xargs apt -y install < /app/dep/packages.txt

rosdep update

pip3 install --upgrade pip 

rosdep install --from-paths /app/src/workspace/src --ignore-src  -r -y -q

find /app/src/workspace/src -name "dependencies.txt" -exec pip3 install -r {} \;
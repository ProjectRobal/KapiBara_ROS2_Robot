#!/bin/bash

# cleanup() {
#     echo "Caught termination signal. Cleaning up..."
#     kill -SIGINT -1   # Sends SIGINT to all subprocesses
#     wait
#     echo "All processes terminated. Exiting."
#     exit 0
# }

# trap cleanup SIGINT SIGTERM

# udevadm control --reload 

# sudo apt-get update

# rosdep update

# pip3 install --upgrade pip 

# rosdep install --from-paths /app/src/workspace/src --ignore-src  -r -y -q

# find /app/src/workspace/src -name "dependencies.txt" -exec pip3 install -r {} \;

# # xargs apt -y install < /app/dep/packages.txt

cleanup() {
    echo "Caught termination signal. Cleaning up..."
    sleep 2
    echo "Cleanup done. Exiting."
    exit 0
}

# # Trap SIGINT
# trap cleanup SIGINT

echo "Ready to go!!"

# /run.sh &


# Simulate long-running process
while true; do
    sleep 1
done

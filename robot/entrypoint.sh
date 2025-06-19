#!/bin/bash


# udevadm control --reload 

# # xargs apt -y install < /app/dep/packages.txt

cleanup() {
    echo "Caught termination signal. Cleaning up..."
    # kill -s SIGINT -$$
    sleep 20
    # wait
    echo "Cleanup done. Exiting."
    exit 0
}

# # Trap SIGINT
trap cleanup SIGINT

echo "Ready to go!!"

# /run.sh &

# exec /run_sim.sh

# /run_sim.sh &

# wait

# Simulate long-running process
while true; do
    sleep 1
done

#!/bin/bash
clear

xhost local:docker

arg=$1


if [ "$arg" = "start" ]; then

docker compose -f compose.yml up --remove-orphans

elif [ "$arg" = "purge" ]; then

docker compose -f compose.yml down

elif [ "$arg" = "rebuild" ]; then

docker compose -f compose.yml build --no-cache

elif [ "$arg" = "build" ]; then

docker compose -f compose.yml build

elif [ "$arg" = "cmd" ]; then

docker compose -f compose.yml exec gazebo /app/cmd/run_cmd.sh "${@:2}"

elif [ "$arg" = "logs" ]; then

docker compose -f compose.yml logs gazebo

elif [ "$arg" = "exec" ]; then

docker compose -f compose.yml exec gazebo "${@:2}"

elif [ "$arg" = "run" ]; then

docker compose -f compose.yml run gazebo /app/cmd/run_cmd.sh "${@:2}"

elif [ "$arg" = "bash" ]; then

docker compose -f compose.yml run gazebo bash

elif [ "$arg" = "debug" ]; then

docker compose -f compose.yml exec gazebo bash

elif [ "$arg" = "gazebo" ]; then

docker compose -f compose.yml exec gazebo /app/cmd/run_cmd.sh ros2 launch gazebo_ros gazebo.launch.py

elif [ "$arg" = "topics" ]; then

docker compose -f compose.yml exec gazebo /app/cmd/run_cmd.sh ros2 topic list -t

elif [ "$arg" = "echo" ]; then

docker compose -f compose.yml exec gazebo /app/cmd/run_cmd.sh ros2 topic echo "${@:2}"

elif [ "$arg" = "info" ]; then

docker compose -f compose.yml exec gazebo /app/cmd/run_cmd.sh ros2 topic info "${@:2}"

elif [ "$arg" = "publish" ]; then

docker compose -f compose.yml exec gazebo /app/cmd/run_cmd.sh ros2 topic pub --once "${@:2}"

fi

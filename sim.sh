#!/bin/bash
clear

xhost local:docker

arg=$1


if [ "$arg" = "start" ]; then

docker compose -f compose_sim.yml up --remove-orphans

elif [ "$arg" = "compile" ]; then

# docker compose -f build_compose_sim.yml up
docker compose -f compose_sim.yml run gazebo /app/cmd/build_packages.sh

elif [ "$arg" = "purge" ]; then

docker compose -f compose_sim.yml down

elif [ "$arg" = "rebuild" ]; then

docker compose -f compose_sim.yml build --no-cache

elif [ "$arg" = "build" ]; then

docker compose -f compose_sim.yml build

elif [ "$arg" = "cmd" ]; then

docker compose -f compose_sim.yml exec gazebo /app/cmd/run_cmd.sh "${@:2}"

elif [ "$arg" = "logs" ]; then

docker compose -f compose_sim.yml logs gazebo

elif [ "$arg" = "exec" ]; then

docker compose -f compose_sim.yml exec gazebo "${@:2}"

elif [ "$arg" = "run" ]; then

docker compose -f compose_sim.yml run gazebo /app/cmd/run_cmd.sh "${@:2}"

elif [ "$arg" = "bash" ]; then

docker compose -f compose_sim.yml run gazebo bash

elif [ "$arg" = "debug" ]; then

docker compose -f compose_sim.yml exec gazebo bash

elif [ "$arg" = "topics" ]; then

docker compose -f compose_sim.yml exec gazebo /app/cmd/run_cmd.sh ros2 topic list -t

elif [ "$arg" = "echo" ]; then

docker compose -f compose_sim.yml exec gazebo /app/cmd/run_cmd.sh ros2 topic echo "${@:2}"

elif [ "$arg" = "info" ]; then

docker compose -f compose_sim.yml exec gazebo /app/cmd/run_cmd.sh ros2 topic info "${@:2}"

elif [ "$arg" = "publish" ]; then

docker compose -f compose_sim.yml exec gazebo /app/cmd/run_cmd.sh ros2 topic pub --once "${@:2}"

fi

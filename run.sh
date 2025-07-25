#!/bin/bash
clear

arg=$1


if [ "$arg" = "start" ]; then

docker compose -f compose.yml up -t 3600

elif [ "$arg" = "start_ros" ]; then

docker compose -f compose_sim.yml exec -t robot /run.sh

elif [ "$arg" = "start_bg" ]; then

docker compose -f compose.yml up -d

elif [ "$arg" = "clean" ]; then

rm -R robot/src/workspace/build
rm -R robot/src/workspace/install

elif [ "$arg" = "stop" ]; then

docker compose -f compose.yml stop -t 3600

elif [ "$arg" = "compile" ]; then

docker compose -f compose.yml exec robot /app/cmd/build_packages.sh gazebo_ros2_control

elif [ "$arg" = "purge" ]; then

docker compose -f compose.yml down

elif [ "$arg" = "rebuild" ]; then

docker compose -f compose.yml build --no-cache

elif [ "$arg" = "build" ]; then

docker compose -f compose.yml build

elif [ "$arg" = "cmd" ]; then

docker compose -f compose.yml exec robot /app/cmd/run_cmd.sh "${@:2}"

elif [ "$arg" = "update" ]; then

docker compose -f compose.yml exec robot /app/cmd/update.sh

elif [ "$arg" = "logs" ]; then

docker compose -f compose.yml logs robot

elif [ "$arg" = "exec" ]; then

docker compose -f compose.yml exec robot "${@:2}"

elif [ "$arg" = "run" ]; then

docker compose -f compose.yml run robot /app/cmd/run_cmd.sh "${@:2}"

elif [ "$arg" = "debug" ]; then

docker compose -f compose.yml exec robot bash

elif [ "$arg" = "topics" ]; then

docker compose -f compose.yml exec robot /app/cmd/run_cmd.sh ros2 topic list -t

elif [ "$arg" = "echo" ]; then

docker compose -f compose.yml exec robot /app/cmd/run_cmd.sh ros2 topic echo "${@:2}"

elif [ "$arg" = "info" ]; then

docker compose -f compose.yml exec robot /app/cmd/run_cmd.sh ros2 topic info "${@:2}"

elif [ "$arg" = "publish" ]; then

docker compose -f compose.yml exec robot /app/cmd/run_cmd.sh ros2 topic pub --once "${@:2}"

else

echo "start - start a container"
echo "stop - stop a container"
echo "compile - build all packages"
echo "purge - remove containers"
echo "update - update image dependencies"
echo "clean - remove build cache"
echo "rebuild - build image without caches"
echo "build - build image"
echo "cmd - run command in working container, in ros2 workspace"
echo "logs - show container logs"
echo "exec - general run command in working container"
echo "run - run command in container"
echo "debug - enter containter shell"
echo "topics - list topics"
echo "echo - show messages published on topic"
echo "info - show topic info"
echo "publish - publish message into specific topic"

fi

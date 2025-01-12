#!/bin/bash
clear

arg=$1


if [ "$arg" = "start" ]; then

docker compose -f compose.yml up --remove-orphans

elif [ "$arg" = "compile" ]; then

# docker compose -f build_compose.yml up
docker compose -f compose.yml run robot /app/cmd/build_packages.sh

elif [ "$arg" = "purge" ]; then

docker compose -f compose.yml down

elif [ "$arg" = "rebuild" ]; then

docker compose -f compose.yml build --no-cache

elif [ "$arg" = "build" ]; then

docker compose -f compose.yml build

elif [ "$arg" = "cmd" ]; then

docker compose -f compose.yml exec robot /app/cmd/run_cmd.sh "${@:2}"

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
echo "compile - build all packages"
echo "purge - remove containers"
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

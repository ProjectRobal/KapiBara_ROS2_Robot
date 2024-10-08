#!/bin/bash
clear

arg=$1


if [ "$arg" = "start" ]; then

docker compose -f compose.yml up --remove-orphans

elif [ "$arg" = "compile" ]; then

docker compose -f build_compose.yml up

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

elif [ "$arg" = "bash" ]; then

docker compose -f compose.yml run robot bash

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

fi

#!/bin/bash

xhost +

while getopts ":b" opt; do
  case $opt in
    b)
      build_flag=true
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
  esac
done

# Check if "docker compose" is available, use it if available, otherwise use "docker-compose"
if docker compose --version > /dev/null 2>&1; then
  docker_command="docker compose"
else
  echo "docker compose is not available, using docker-compose instead"
  docker_command="docker-compose"
fi

if [ "$build_flag" = true ]; then
  sudo $docker_command build
fi

sudo $docker_command run --rm code
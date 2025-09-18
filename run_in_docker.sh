#! /usr/bin/bash

xhost +local:root

if nvidia-smi &>/dev/null; then
    docker compose -f ./docker/docker-compose_gpu.yml build
    docker compose -f ./docker/docker-compose_gpu.yml up
else
    docker compose -f ./docker/docker-compose.yml build
    docker compose -f ./docker/docker-compose.yml up
fi

xhost -local:root
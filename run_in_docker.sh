#! /usr/bin/bash

xhost +local:root
docker compose -f ./docker/docker-compose.yml build
if nvidia-smi &>/dev/null; then
    docker compose -f ./docker/docker-compose_gpu.yml up
else
    docker compose -f ./docker/docker-compose.yml up
fi

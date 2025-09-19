#! /usr/bin/bash

xhost +local:root

DO_BUILD=false

# Controllo argomenti
if [[ "$1" == "--build" ]]; then
    DO_BUILD=true
fi

if nvidia-smi &>/dev/null; then
    if $DO_BUILD; then
        docker compose -f ./docker/docker-compose_gpu.yml build
    fi
    docker compose -f ./docker/docker-compose_gpu.yml up
else
    if $DO_BUILD; then
        docker compose -f ./docker/docker-compose.yml build
    fi
    docker compose -f ./docker/docker-compose.yml up
fi

xhost -local:root

#! /usr/bin/bash

xhost +local:root
docker compose -f ./docker/docker-compose.yml build
docker compose -f ./docker/docker-compose.yml up
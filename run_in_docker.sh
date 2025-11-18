#! /usr/bin/bash

xhost +local:root

DO_BUILD=false
SCRIPT_ARGS=()

TARGET_SERVICE="ros2-gazebo"   # <-- set this to the service that runs the script

# Parse arguments
if [[ "$1" == "--build" ]]; then
    DO_BUILD=true
    shift                     # remove --build
    SCRIPT_ARGS=("$@")        # collect the rest as script args
fi

# Choose compose file based on GPU availability
if nvidia-smi &>/dev/null; then
    COMPOSE_FILE="./docker/docker-compose_gpu.yml"
else
    COMPOSE_FILE="./docker/docker-compose.yml"
fi

# Build if needed
if $DO_BUILD; then
    docker compose -f "$COMPOSE_FILE" build
fi

# Run container and pass args to the script inside it
docker compose -f "$COMPOSE_FILE" up "$TARGET_SERVICE" -- "${SCRIPT_ARGS[@]}"

xhost -local:root
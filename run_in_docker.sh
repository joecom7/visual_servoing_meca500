#! /usr/bin/bash

xhost +local:root

DO_BUILD=false
START_DRIVER=false
SCRIPT_ARGS=()

TARGET_SERVICE="ros2-gazebo"   # <-- set this to the service that runs the script

# -------------------------------
# Parse arguments
# -------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --build)
            DO_BUILD=true
            shift
            ;;
        --start-driver)
            START_DRIVER=true
            shift
            ;;
        *)
            SCRIPT_ARGS+=("$1")
            shift
            ;;
    esac
done

# -------------------------------
# Choose compose file based on GPU
# -------------------------------
if nvidia-smi &>/dev/null; then
    COMPOSE_FILE="./docker/docker-compose_gpu.yml"
else
    COMPOSE_FILE="./docker/docker-compose.yml"
fi

# -------------------------------
# Build if requested
# -------------------------------
if $DO_BUILD; then
    docker compose -f "$COMPOSE_FILE" build
fi


# -------------------------------
# Start driver if requested
# -------------------------------
if $START_DRIVER; then
    echo "Launching driver terminal..."
    
    # Start driver in a new terminal, record PID
    gnome-terminal -- bash -c "sudo -E -S ~/prova_xenomai_ws/run_driver.sh" &
    DRIVER_PID=$!
    
    echo "Driver launched with terminal PID: $DRIVER_PID"
fi


# -------------------------------
# Run container
# -------------------------------
docker compose -f "$COMPOSE_FILE" up "$TARGET_SERVICE" -- "${SCRIPT_ARGS[@]}"

# -------------------------------
# After container stops: stop driver
# -------------------------------
if $START_DRIVER; then
    echo "Stopping driver… sending Ctrl+C"

    # Send SIGINT (the programmatic equivalent of Ctrl+C)
    kill -SIGINT "$DRIVER_PID" 2>/dev/null

    # Give it a moment to exit cleanly
    sleep 1

    # If still alive, force-stop
    if kill -0 "$DRIVER_PID" 2>/dev/null; then
        echo "Driver did not exit, killing..."
        kill -9 "$DRIVER_PID" 2>/dev/null
    fi
fi

xhost -local:root

#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
cd /root/ws/
colcon build

source ./install/setup.bash

source /venv/bin/activate

ros2 launch meca500_launch sim_test.launch.py

exec "$@"
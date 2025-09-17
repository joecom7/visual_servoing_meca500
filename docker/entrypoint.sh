#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /root/ws/install/setup.bash

ros2 launch meca500_launch sim_test.launch.py

exec "$@"
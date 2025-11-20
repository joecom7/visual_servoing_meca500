set -e
source /opt/ros/jazzy/setup.bash
cd /root/ws/
colcon build

source ./install/setup.bash

source /venv/bin/activate
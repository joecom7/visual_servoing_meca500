FROM ros:jazzy

RUN apt-get update && apt-get install -y \
    curl lsb-release gnupg

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt-get update && apt-get install -y  gz-harmonic

RUN apt-get update && apt-get install -y ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-image ros-jazzy-cv-bridge

RUN apt-get install -y python3-venv

# Create a virtual environment in /venv
RUN python3 -m venv /venv

RUN /venv/bin/pip uninstall -y numpy

# Activate venv and install packages
RUN /venv/bin/pip install --upgrade pip && \
    /venv/bin/pip install "numpy<2.0" ultralytics

RUN /venv/bin/pip install catkin_pkg empy lark

RUN apt-get install -y ros-jazzy-moveit

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
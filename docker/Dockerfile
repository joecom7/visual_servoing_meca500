FROM ros:jazzy

RUN apt-get update && apt-get install -y \
    curl lsb-release gnupg

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt-get update && apt-get install -y  gz-harmonic

RUN apt-get update && apt-get install -y ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-image ros-jazzy-cv-bridge ros-jazzy-moveit

RUN apt-get update && apt-get install -y python3-venv virtualenv

# Create a virtual environment in /venv
RUN virtualenv -p python3 /venv

RUN touch /venv/COLCON_IGNORE

RUN /venv/bin/pip uninstall -y numpy

# Activate venv and install packages

COPY requirements.txt /requirements.txt

RUN /venv/bin/pip install --upgrade pip && \
    /venv/bin/pip install -r /requirements.txt

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
FROM ros:jazzy

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8

# ----------------------------
# 1. Locale setup
# ----------------------------
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# ----------------------------
# 2. Add ROS 2 apt repository 
#    (using ros2-apt-source — exactly as tutorial)
# ----------------------------
RUN apt-get update && apt-get install -y \
    software-properties-common curl gnupg2 lsb-release

RUN add-apt-repository universe

# dynamically get the latest ros2-apt-source package
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb

# ----------------------------
# 3. Install development tools & ROS tools
# ----------------------------
RUN apt-get update && apt-get install -y \
    python3-flake8-docstrings \
    python3-pip \
    python3-pytest-cov \
    ros-dev-tools \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pytest-repeat \
    python3-pytest-rerunfailures

# ----------------------------
# 4. Create workspace & get ROS 2 Humble sources
# ----------------------------
WORKDIR /root/ros2_humble
RUN mkdir -p src

RUN curl -sSL https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos -o ros2.repos && \
    vcs import --input ros2.repos src

# ----------------------------
# 5. rosdep init, update, install dependencies
# ----------------------------
RUN apt-get update && apt-get upgrade -y

RUN rosdep init || true
RUN rosdep update

# Important: skip problematic keys exactly like tutorial
RUN rosdep install --from-paths src --ignore-src -y \
    --os=ubuntu:jammy \
    --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers ignition-cmake2 ignition-math6 ignition-tools gazebo gazebo-dev"
# ----------------------------
# 6. Build ROS 2 Humble
# ----------------------------
RUN colcon build --symlink-install

# ----------------------------
# 7. Environment setup
# ----------------------------
RUN echo "source /root/ros2_humble/install/local_setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash"]

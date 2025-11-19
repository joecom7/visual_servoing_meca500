FROM ros:jazzy

RUN apt-get update && apt-get install -y \
    curl lsb-release gnupg

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt-get update && apt-get install -y  gz-harmonic ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-image ros-jazzy-cv-bridge ros-jazzy-moveit python3-venv virtualenv wget fuse ros-jazzy-realtime-tools

# RUN cd /tmp && \
#     wget https://source.denx.de/Xenomai/xenomai/-/archive/v3.2.4/xenomai-v3.2.4.tar.gz && \
#     tar zxf xenomai-v3.2.4.tar.gz && \
#     cd xenomai-v3.2.4 && \
#     ./scripts/bootstrap && \
#     ./configure \
#         --with-pic \
#         --with-core=cobalt \
#         --enable-smp \
#         --disable-tls \
#         --enable-dlopen-libs \
#         --disable-clock-monotonic-raw && \
#     make -j"$(nproc)" && \
#     make install && \
#     rm -rf /tmp/xenomai-v3.2.4 /tmp/xenomai-v3.2.4.tar.gz

# RUN echo "/usr/xenomai/lib" > /etc/ld.so.conf.d/xenomai.conf && ldconfig

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
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=30
CMD ["bash"]

RUN apt-get update && apt-get install -y ros-jazzy-rmw-cyclonedds-cpp
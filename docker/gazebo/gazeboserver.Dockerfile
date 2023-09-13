# ================= Dependencies ===================
FROM ros:humble AS base

RUN apt-get update && apt-get install -y curl && \
    rm -rf /var/lib/apt/lists/*

# RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys A4B469963BF863CC
RUN apt-get update && apt-get install ffmpeg libsm6 libxext6 -y

RUN apt install -y lsb-release wget gnupg
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get -y update
RUN apt-get -y install ros-${ROS_DISTRO}-ros-gz ignition-fortress
RUN apt-get -y install ros-humble-velodyne-gazebo-plugins
RUN echo $GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib

# fix user permissions for mounted volumes
COPY docker/fixuid_setup.sh /project/fixuid_setup.sh
RUN /project/fixuid_setup.sh
USER docker:docker

ENV DEBIAN_FRONTEND noninteractive
RUN sudo chsh -s /bin/bash
ENV SHELL=/bin/bash

# ================= Repositories ===================
FROM base as repo

USER docker:docker
WORKDIR /home/docker

ENV DEBIAN_FRONTEND interactive

COPY docker/wato_ros_entrypoint.sh /home/docker/wato_ros_entrypoint.sh
COPY docker/ros_entrypoint.sh /home/docker/ros_entrypoint.sh
COPY docker/.bashrc /home/docker/.bashrc

ENTRYPOINT ["/usr/local/bin/fixuid", "-q", "/home/docker/ros_entrypoint.sh"]

# CMD ["sleep", "infinity"]
# CMD ["ign", "launch", "launch/sim.ign"]

CMD ["ros2", "launch", "src/gazebo/launch/sim.launch.py"]

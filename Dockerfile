FROM ubuntu:focal

# ros-core
# Ref: https://github.com/osrf/docker_images/blob/master/ros/noetic/ubuntu/focal/ros-core/Dockerfile
# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    # ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*
# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*
# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list
# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO noetic
# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-core=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# ros-base
# Ref: https://github.com/osrf/docker_images/blob/master/ros/noetic/ubuntu/focal/ros-base/Dockerfile
# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    && rm -rf /var/lib/apt/lists/*
# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO
# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# ros-robot
# Ref: https://github.com/osrf/docker_images/blob/master/ros/noetic/ubuntu/focal/robot/Dockerfile
# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-robot=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# ros-desktop
# Ref: https://github.com/osrf/docker_images/blob/master/ros/noetic/ubuntu/focal/desktop/Dockerfile
# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# ros-desktop-full
# Ref: https://github.com/osrf/docker_images/blob/master/ros/noetic/ubuntu/focal/desktop-full/Dockerfile
# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# ros-core
# Ref: https://github.com/osrf/docker_images/blob/master/ros/noetic/ubuntu/focal/ros-core/Dockerfile
# setup entrypoint
COPY ./thirdparty/ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

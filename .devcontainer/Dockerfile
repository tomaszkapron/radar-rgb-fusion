FROM ubuntu:jammy
ARG WORKSPACE
ENV WORKSPACE=${WORKSPACE}

ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV DEBIAN_FRONTEND=
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV NVIDIA_VISIBLE_DEVICES=all
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages
ENV QT_X11_NO_MITSHM=1
ENV ROS_DISTRO=humble
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2


# install packages & setup locales
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    apt-utils \
    bash-completion \
    build-essential \
    ca-certificates \
    curl \
    dirmngr \
    gdb \
    git \
    gnupg2 \
    keyboard-configuration \
    locales \
    lsb-release \
    python3-pip \
    software-properties-common \
    vim \
    wget \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8; dpkg-reconfigure -f noninteractive locales

ENV LANG="en_US.UTF-8" \
    LANGUAGE="en_US.UTF-8" \
    LC_ALL="en_US.UTF-8"


# install ROS2 & init rosdep
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-humble-ros-base \
    ros-humble-rviz2 \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* \
    && /bin/bash -c 'source /opt/ros/humble/setup.bash; rosdep init'


# install python packages
RUN python3 -m pip --no-cache-dir install \
    setuptools==58.2.0


# customize bash
COPY bashrc-git-prompt /tmp/
RUN curl -L https://raw.github.com/git/git/master/contrib/completion/git-prompt.sh >> /etc/skel/.bashrc \
    && cat /tmp/bashrc-git-prompt >> /etc/skel/.bashrc \
    && rm -rf /tmp/bashrc-git-prompt \
    && echo "source /opt/ros/humble/setup.bash" >> /etc/skel/.bashrc


# install your custom packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-controller-interface \
    ros-humble-moveit \
    ros-humble-moveit-resources \
    ros-humble-moveit-visual-tools \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-robot-controllers \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ros-testing \
    ros-humble-rviz-visual-tools


# install your custom python packages
# RUN python3 -m pip --no-cache-dir install \
#    package_name_1 \
#    package_name_2


# set up entrypoint
COPY entrypoint /entrypoint
ENTRYPOINT ["/entrypoint"]

CMD ["/bin/bash"]
LABEL maintainer="Tomasz Kapron (tomasz.kapron@student.put.poznan.pl)"

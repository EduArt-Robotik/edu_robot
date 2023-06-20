FROM ubuntu:jammy

# using bash
SHELL ["/bin/bash", "-c"]
ENV UBUNTU_DISTRO jammy
ENV ROS_DISTRO humble
ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

RUN \
    apt-get update \
    && apt-get install -q -y --no-install-recommends \
    apt-utils \
    software-properties-common \
    nano \
    curl \
    gnupg \
    git \
    build-essential \
    sudo \
########################################################
# Install ROS
## Get package archive key and ROS source list entry.
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_DISTRO main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update \
########################################################
# Install ROS packages
    && apt-get install -q -y --no-install-recommends \
    ros-$ROS_DISTRO-ros-base \
    python3-colcon-common-extensions \
    software-properties-common \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    ros-$ROS_DISTRO-fastrtps

# write source commands to .bashrc -> no need to source afterwards
RUN \
########################################################
# Add non root user for development.
    useradd --create-home --shell /bin/bash $USER \
    && echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && /sbin/usermod -aG sudo $USER \
########################################################
# Add exports and sources to bash rc for easy handling.
    && echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /home/$USER/.bashrc \
    && echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/fastrtps_profile.xml" >> /home/$USER/.bashrc \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USER/.bashrc \
    && echo "source /home/$USER/ipcbot_ws/install/setup.bash" >> /home/$USER/.bashrc \
########################################################
# Clean up
    && apt-get clean

# work with user user
USER $USER

# set up iotbot workspace
RUN mkdir -p /home/$USER/ipcbot_ws/src

COPY ./edu_robot /home/$USER/ipcbot_ws/src/edu_robot
COPY ./fastrtps_profile.xml /home/$USER/ipcbot_ws

WORKDIR /home/$USER/ipcbot_ws

# build EduArt packages nodes
RUN export RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && MAKEFLAGS="-j2" colcon build --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# launch control node and joy interpreter
CMD export RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    && export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/ipcbot_ws/fastrtps_profile.xml \
    && source /home/user/ipcbot_ws/install/setup.bash \
    && ros2 launch edu_robot eduard-ipc127e.launch.py

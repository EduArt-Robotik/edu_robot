FROM ros:humble-ros-base-jammy

# using bash
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO humble
ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

RUN \
    apt-get update \
    && apt-get install -y software-properties-common

# install Joystick Driver for ROS2
RUN apt-get update \
    && sudo apt-get install -y \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    ros-$ROS_DISTRO-fastrtps

# set up iotbot workspace
RUN mkdir -p /home/$USER/ipcbot_ws/src/iotbot

COPY ./edu_robot /home/$USER/ipcbot_ws/src/edu_robot
COPY ./fastrtps_profile.xml /home/$USER/ipcbot_ws

WORKDIR /home/$USER/ipcbot_ws

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

# build EduArt packages nodes
RUN export RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && MAKEFLAGS="-j2" colcon build --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# launch control node and joy interpreter
CMD export RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    && export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ipcbot_ws/fastrtps_profile.xml \
    && source /home/ipcbot_ws/install/setup.bash \
    && ros2 launch edu_robot eduard-ipc127e.launch.py

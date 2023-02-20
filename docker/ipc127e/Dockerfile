FROM ros:galactic-ros-base-focal

# using bash
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO galactic

RUN \
    apt-get update \
    && apt-get install -y software-properties-common

# install mraa lib for UART communication in shield node
RUN add-apt-repository ppa:mraa/mraa \
    && apt-get update \
    && apt-get install -y libmraa2 libmraa-dev libmraa-java python-mraa python3-mraa mraa-tools

# install Joystick Driver for ROS2 foxy
RUN sudo apt-get install -y \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    ros-$ROS_DISTRO-fastrtps

# set up iotbot workspace
RUN mkdir -p /home/ipcbot_ws/src/iotbot

COPY ./edu_robot /home/ipcbot_ws/src/edu_robot
COPY ./edu_robot_control /home/ipcbot_ws/src/edu_robot_control
COPY ./fastrtps_profile.xml /home/ipcbot_ws

WORKDIR /home/ipcbot_ws

# build EduArt packages nodes
RUN export RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && MAKEFLAGS="-j2" colcon build --symlink-install --executor sequential

# write source commands to .bashrc -> no need to source afterwards
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc \
    && echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ipcbot_ws/fastrtps_profile.xml" >> ~/.bashrc \
    && echo "source /home/ipcbot_ws/install/setup.bash" >> ~/.bashrc

# launch control node and joy interpreter
CMD source ~/.bashrc \
    && source /home/ipcbot_ws/install/setup.bash \
    && (ros2 launch edu_robot_control robot_remote_control.launch.py \
    & ros2 launch edu_robot eduard-ipc127e.launch.py)

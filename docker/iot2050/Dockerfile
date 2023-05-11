FROM ros:humble-ros-base-jammy

# using bash
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO humble
ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

RUN \
    apt-get update \
    && apt-get install -y software-properties-common

# install mraa lib for UART communication in shield node
RUN add-apt-repository ppa:mraa/mraa \
    && apt-get update \
    && apt-get install -y libmraa2 libmraa-dev libmraa-java python-mraa python3-mraa mraa-tools

# install Joystick Driver for ROS2 foxy
RUN sudo apt-get install -y \
#    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    ros-$ROS_DISTRO-fastrtps

# set up iotbot workspace
RUN mkdir -p /home/iotbot_ws/src/iotbot

COPY ./edu_robot /home/iotbot_ws/src/edu_robot
COPY ./edu_robot_control /home/iotbot_ws/src/edu_robot_control
COPY ./joystick_drivers /home/iotbot_ws/src/joystick_drivers

WORKDIR /home/iotbot_ws

# build EduArt packages nodes
RUN export RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && MAKEFLAGS="-j1" colcon build --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# write source commands to .bashrc -> no need to source afterwards
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc \
    && echo "source /home/iotbot_ws/install/setup.bash" >> ~/.bashrc

# launch control node and joy interpreter
CMD source /home/iotbot_ws/install/setup.bash \
    && (ros2 launch edu_robot_control robot_remote_control.launch.py \
    & ros2 launch edu_robot eduard-iot2050.launch.py)

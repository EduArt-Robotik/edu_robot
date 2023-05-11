FROM ros:humble-ros-base-jammy

# using bash
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO humble
ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

RUN \
# install Joystick Driver for ROS2 foxy
    apt-get update \
    && apt-get install -y \
    software-properties-common \
    # ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    ros-$ROS_DISTRO-fastrtps \
    ros-$ROS_DISTRO-desktop \
    ros-$ROS_DISTRO-xacro

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# write source commands to .bashrc -> no need to source afterwards
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc \
########################################################
# Add non root user for development
    && useradd --create-home --shell /bin/bash $USER \
    && echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && /sbin/usermod -aG sudo $USER \
    && echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /home/$USER/.bashrc \
    && echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/fastrtps_profile.xml" >> /home/$USER/.bashrc \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USER/.bashrc \
    && echo "source /home/$USER/eduard_ws/install/setup.bash" >> /home/$USER/.bashrc \
########################################################
# Clean up
    && apt-get clean

COPY ./edu_robot /home/$USER/eduard_ws/src/edu_robot
COPY ./edu_robot_control /home/$USER/eduard_ws/src/edu_robot_control
COPY ./joystick_drivers /home/$USER/eduard_ws/src/joystick_drivers
COPY ./fastrtps_profile.xml /home/$USER

WORKDIR /home/$USER/eduard_ws

RUN export RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build --packages-select edu_robot edu_robot_control --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

CMD source /home/$USER/eduard_ws/install/setup.bash \
    && ros2 launch edu_robot_control eduard-monitor.launch.py

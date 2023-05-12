FROM ros:humble-ros-base-jammy

# using bash
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO humble
ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

# install mraa lib for UART communication in shield node
RUN apt-get update \
    && apt-get install -y software-properties-common git build-essential swig3.0 python2-dev python2 python-dev-is-python3 libnode-dev cmake libjson-c-dev \
    # todo master brauch is not good, but the only one how compiles.
    && git clone --branch master https://github.com/eclipse/mraa.git \
    && cd mraa \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install

# install Joystick Driver for ROS2 foxy
RUN sudo apt-get install -y \
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
    # && echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/fastrtps_profile.xml" >> /home/$USER/.bashrc \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USER/.bashrc \
    && echo "source /home/$USER/eduart_ws/install/setup.bash" >> /home/$USER/.bashrc \
########################################################
# Clean up
    && apt-get clean

# set up iotbot workspace
RUN mkdir -p /home/$USER/iotbot_ws/src/

COPY ./edu_robot /home/$USER/iotbot_ws/src/edu_robot
# COPY ./fastrtps_profile.xml /home/$USER

WORKDIR /home/$USER/iotbot_ws

# build EduArt packages nodes
RUN source /home/$USER/.bashrc \
    && MAKEFLAGS="-j1" colcon build --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# launch control node and joy interpreter
CMD source /home/$USER/.bashrc \
    && ros2 launch edu_robot eduard-iot2050.launch.py

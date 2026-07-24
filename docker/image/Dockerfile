FROM eduartrobotik/eduart-ros-base:jazzy-1.2.3

ENV USER=user
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# install mraa lib for UART communication in shield node
USER root
RUN apt-get update \
    && apt-get install -y \
      software-properties-common \
      git \
      build-essential \
      swig3.0 \
      python-dev-is-python3 \
      libnode-dev \
      cmake \
      libjson-c-dev \
      libgpiod-dev \
      libusb-1.0-0-dev \
    && apt-get install -y \
      ros-$ROS_DISTRO-hardware-interface \
    # todo master brauch is not good, but the only one how compiles.
    && git clone --branch master https://github.com/eclipse/mraa.git \
    && cd mraa \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install \
    # hack: was the fastest and most fluffy way
    && cp /usr/local/lib/libmraa.so* /usr/lib

# libsensoring for the distance sensor and lighting for raspberry pi
RUN cd /tmp \
    && git clone --branch develop https://github.com/EduArt-Robotik/edu_lib_sensorring.git \
    && cd edu_lib_sensorring \
    && git submodule sync --recursive \
    && git -c url."https://github.com/".insteadOf="git@github.com:" \
       -c url."https://github.com/".insteadOf="ssh://git@github.com/" \
       submodule update --init --recursive \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install \
    && rm -rf /tmp/edu_lib_sensorring

# set up user space
USER $USER

# set up ros workspace
RUN mkdir -p /home/$USER/ros/src/
COPY ./edu_robot /home/$USER/ros/src/edu_robot
WORKDIR /home/$USER/ros

# build EduArt packages nodes
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# add entry point
COPY --chown=$USER ./entrypoint /home/$USER/
RUN chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]

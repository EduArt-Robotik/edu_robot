FROM eduartrobotik/eduart-ros-base:jazzy-1.0.1

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
    && apt-get install -y \
      ros-$ROS_DISTRO-hardware-interface \
    # todo master brauch is not good, but the only one how compiles.
    && git clone --branch master https://github.com/eclipse/mraa.git \
    && cd mraa \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install \
    # hack: was the fastest and most fluffy way
    && cp /usr/local/lib/libmraa.so* /usr/lib

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

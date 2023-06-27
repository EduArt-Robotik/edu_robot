FROM eduartrobotik/eduart-robot:0.2.1

ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

# add fastrtps profile
# todo: check if it is still needed!
COPY ./fastrtps_profile.xml /home/$USER/ipcbot_ws

# launch control node and joy interpreter
CMD export RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    && export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/ipcbot_ws/fastrtps_profile.xml \
    && source /home/user/ipcbot_ws/install/setup.bash \
    && ros2 launch edu_robot eduard-ipc127e.launch.py

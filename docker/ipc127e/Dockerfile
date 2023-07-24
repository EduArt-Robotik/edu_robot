FROM eduartrobotik/eduart-robot:0.3.0

ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

COPY ./ipc_cyclone_dds_profile.xml /home/$USER/

ENV CYCLONEDDS_URI=/home/$USER/ipc_cyclone_dds_profile.xml

# launch control node and joy interpreter
CMD echo "launching eduard ipc127e robot control software" \
    # && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    # && export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/ros/fastrtps_profile.xml \
    && source /home/user/ros/install/setup.bash \
    && ros2 launch edu_robot eduard-ipc127e.launch.py

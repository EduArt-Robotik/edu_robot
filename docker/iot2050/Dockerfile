FROM eduartrobotik/eduart-robot:0.3.0

ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

COPY ./iot_cyclone_dds_profile.xml /home/$USER/

ENV CYCLONEDDS_URI=/home/$USER/iot_cyclone_dds_profile.xml

# launch control node and joy interpreter
CMD echo "launching eduard iot2050 robot control software" \
    # && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/user/ros/install/setup.bash \    
    && ros2 launch edu_robot eduard-iot2050.launch.py

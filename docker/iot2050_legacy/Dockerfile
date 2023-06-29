FROM eduartrobotik/eduart-robot-control:0.2.0

ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

# launch control node and joy interpreter
CMD export RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/user/iotbot_ws/install/setup.bash \    
    && (ros2 launch edu_robot eduard-iot2050.launch.py & ros2 launch edu_robot_control robot_remote_control_iot2050.launch.py)

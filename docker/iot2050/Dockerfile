FROM eduartrobotik/eduart-robot:0.3.2

ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

# in case cyclone is used this profile is needed
COPY ./iot_cyclone_dds_profile.xml /home/$USER/
ENV CYCLONEDDS_URI=/home/$USER/iot_cyclone_dds_profile.xml

# add entry point
COPY --chown=$USER ./entrypoint /home/$USER/
RUN sudo chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]

# launch control node and joy interpreter
CMD [ "ros2", "launch", "edu_robot", "eduard-iot2050.launch.py" ]

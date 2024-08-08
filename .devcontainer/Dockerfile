# ROS Distribution we want to use
ARG ROS_DISTRO=rolling

FROM osrf/ros:${ROS_DISTRO}-desktop-full AS dev
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL [ "/bin/bash", "-c"]

RUN echo ubuntu ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/ubuntu && chmod 0440 /etc/sudoers.d/ubuntu

RUN mkdir -p /mobile_robotics_ws
WORKDIR /mobile_robotics_ws

# # Set the ownership of the mms_ws2 workspace to the new user
# RUN chown -R ${UID}:${GID} /mobile_robotics_ws/

# Set the user and source entrypoint in the user's .bashrc file
USER ubuntu
COPY ./entrypoint.sh /
RUN echo "source /entrypoint.sh" >> /home/ubuntu/.bashrc

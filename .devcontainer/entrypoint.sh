#!/bin/bash

## Basic entrypoint for ROS / Colcon Docker containers

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source the workspace, if built
if [ -f /workspaces/mobile-robotics-fyp/install/setup.bash ]
then
  source /workspaces/mobile-robotics-fyp/install/setup.bash
fi

# Update dependencies, if thats what the user wants
printf 'Update dependencies (y/N)? '
read answer
if [ "$answer" != "${answer#[Yy]}" ] ;then
  sudo apt update
  cd /workspaces/mobile-robotics-fyp
  rosdep update
  rosdep install --ignore-src --default-yes --from-path src
fi

# Execute the command passed into this entrypoint
exec "$@"

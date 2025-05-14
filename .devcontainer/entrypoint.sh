#!/bin/bash

# Basic entrypoint for ROS / Colcon Docker containers
# This script is run everytime a user enters the dev container.

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source the workspace, if built
if [ -f /workspaces/mobile-robotics-fyp/install/setup.bash ]
then
  source /workspaces/mobile-robotics-fyp/install/setup.bash
  source /workspaces/mobile-robotics-fyp/install/local_setup.bash
fi

# Update dependencies if required
INIT_FILE="/home/ubuntu/.init"

# Check if the .last_update file exists
if [ -f "$INIT_FILE" ]; then
  echo "update.sh has run one once, run manually if required."
else
  echo "Running update.sh."

  # Run update.sh if .last_update does not exist
  source /workspaces/mobile-robotics-fyp/.devcontainer/update.sh

  # Create .init and store the current time - if update.sh worked
  if [ $? -eq 0 ]; then
    touch "$INIT_FILE"
  fi

fi

# Execute the command passed into this entrypoint
exec "$@"

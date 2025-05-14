#!/bin/bash

# determine the directory of the script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
pushd "$DIR/.." > /dev/null

# Set the workspace directory
COLCON_WS=${COLCON_WS:-$(pwd)}

sudo apt update # Update the cache, since rosdep may not do it, and packages may have been removed since last check
rosdep update
UBUNTU_DISTRO=
if [ "${ROS_DISTRO}" = "jazzy" -o "${ROS_DISTRO}" = "rolling" ]; then
    UBUNTU_DISTRO="noble"
else
    echo "Unknown ROS distribution: ${ROS_DISTRO}"
    exit 1
fi
rosdep install --os=ubuntu:${UBUNTU_DISTRO} --ignore-src --default-yes --from-path src -r


# Install the colcon mixin for the default repositories and the custom mms mixins
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml || true
colcon mixin add mobile_robotics file://${COLCON_WS}/tools/mixin/index.yaml || true
colcon mixin update

popd > /dev/null

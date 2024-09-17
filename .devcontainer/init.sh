#!/bin/bash

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/gazebo.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# sudo curl -sSl https://packages.osrfoundation.org/gazebo.key -o /etc/apt/keyrings/gazebo.gpg
gpg --no-default-keyring --keyring /tmp/keyring.gpg --keyserver hkps://keyserver.ubuntu.com --recv-key 67170598af249743 && sudo gpg --no-default-keyring --keyring /tmp/keyring.gpg --output /etc/apt/keyrings/gazebo.gpg --export
sudo apt update
sudo apt install -y libgz-transport13-dev python3-gz-transport13

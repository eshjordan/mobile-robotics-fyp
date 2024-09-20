# mobile-robotics-fyp
ROS 2 Colcon workspace for use in modelling and testing coverage problems with distributed, lazy agents with event-driven communication.

## Preferred Development Workflow
1. Install Docker.
1. Use VS Code's Dev Containers feature to automatically setup the development environment.
1. If not already installed, add the extension `ms-vscode-remote.remote-containers`.
1. When prompted, open the workspace in the container, and you will be good to go.

## Devcontainer on Windows
1. Install VcXsrv - https://github.com/marchaesen/vcxsrv/releases/latest
1. Append the following directory to your `PATH` environment variable: `C:\Program Files\VcXsrv`

## Local installation, if not using Dev Containers (which is preferred)
Install ROS 2 Rolling Ridley on Ubuntu 24.04

https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html

```bash
locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install ros-dev-tools

sudo apt update
sudo apt upgrade

sudo apt install ros-rolling-desktop-full

echo "source /opt/ros/rolling/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

# Install gazebo transport
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/gazebo.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# sudo curl -sSl https://packages.osrfoundation.org/gazebo.key -o /etc/apt/keyrings/gazebo.gpg
gpg --no-default-keyring --keyring /tmp/keyring.gpg --keyserver hkps://keyserver.ubuntu.com --recv-key 67170598af249743 && sudo gpg --no-default-keyring --keyring /tmp/keyring.gpg --output /etc/apt/keyrings/gazebo.gpg --export
sudo apt update
sudo apt install -y libgz-transport13-dev python3-gz-transport13
```

# Install Webots
https://www.cyberbotics.com/doc/guide/installing-webots

```bash
glxinfo | grep OpenGL # Check the output contains "NVIDIA", "AMD", or "Intel", otherwise install the appropriate hardware accelerated driver

# Check if compiz is running, disabling it can significantly improve Webots rendering performance
ps -A | grep compiz

# If it is, disable compiz desktop effects (somehow using unity-tweak-tool)
sudo apt install unity-tweak-tool unity-lens-applications unity-lens-files
unity-tweak-tool

# Get the Webots keyring and add the repository to the sources list
sudo mkdir -p /etc/apt/keyrings
cd /etc/apt/keyrings
sudo wget -q https://cyberbotics.com/Cyberbotics.asc

echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list
sudo apt update

# Install Webots
sudo apt install webots
```

## Webots ROS 2 Interface
https://docs.ros.org/en/rolling/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html

```bash
sudo apt install ros-rolling-webots-ros2
```

# Workspace Setup
Install clangd for formatting and linting, and all dependencies of the packages in the workspace
```bash
sudo apt update && sudo apt install clangd-16 clang-tidy-16 clang-format-16
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

# Testing
https://cyberbotics.com/doc/guide/epuck?version=R2023a

Use either e-puck2.wbt or e-puck2_server.wbt as the world file
 - e-puck2.wbt: "simple example of collision avoidance behavior using a Braitenberg based controller. This is done using the distance sensors of the e-puck."
 - e-puck2_server.wbt: "robot running a controller that implements a TCP/IP server emulating the e-puck2 Wi-Fi protocol. Allows users to test the behavior of the simulated e-puck2 robot with the TCP/IP Advance Sercom protocol commands."

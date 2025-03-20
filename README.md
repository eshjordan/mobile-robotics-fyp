# mobile-robotics-fyp
ROS 2 Colcon workspace for use in modelling and testing coverage problems with distributed, lazy agents with event-driven communication.

## Cloning this repo
```bash
cd $HOME
git clone https://github.com/eshjordan/mobile-robotics-fyp.git colcon_ws
cd colcon_ws
git submodule init src epuck/esp-idf
git submodule update --recursive src epuck/esp-idf
```

## Development Workflow Setup

### Dev Containers Development Workflow
1. Install Docker.
1. Open `workspace.code-workspace` in VS Code
1. If not already installed, add the extension `ms-vscode-remote.remote-containers`.
1. When prompted, open the workspace in the container (or `Ctrl+Shift P` then look for `Dev Containers: Rebuild and Reopen in Container`), and you will be good to go.
1. The first time you open the console inside the container, when prompted, update the dependencies.

#### Dev Container on Windows
1. Install VcXsrv - https://github.com/marchaesen/vcxsrv/releases/latest
1. Run XLaunch
1. Rebuild and reopen in container

### Local Installation Development Workflow

#### Install ROS 2 Rolling Ridley on Ubuntu 24.04

Follow the steps at https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html

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

#### Install gazebo transport
Install some additional packages required for Gazebo.

```bash
sudo apt update
sudo apt install -y lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install -y libgz-transport13-dev python3-gz-transport13 python3-transforms3d xterm python3-rosdep colcon gz-ionic
```

#### Workspace Setup
Install all dependencies of the packages in the workspace.
```bash
sudo rosdep init
rosdep update
rosdep install --os=ubuntu:noble --from-paths src --ignore-src -r -y
```

## Build Workspace
```bash
cd colcon_ws
colcon build # Optionally, add the --symlink-install option
. install/setup.bash
```

## Launch Launchfiles
After sourcing the `install/setup.bash` script:

```bash
ros2 launch <PACKAGE NAME> <LAUNCHFILE NAME>
```

## E-Puck2 ESP32 Radio Module Firmware
### Build
If the cross-compiler toolchain is not installed, the build script should automatically download this for you.
```bash
cd colcon_ws/epuck/esp-idf
./build.sh
```
### Flash

Connect the E-Puck2 via USB, and power it on.
There should be two new devices present in the `/dev` directory, `/dev/ttyACM0` and `/dev/ttyACM1`.

```bash
cd colcon_ws/epuck/esp-idf
./flash.sh
```

Make sure the mode selection knob is set to `F`. If the firmware immediately crashed, you may need to adjust this.
Sometimes this knob is slightly out of alignment, so try moving to the next closest position.

### Serial Monitor
Use minicom to monitor the ESP32's serial output.
If not installed, run
```bash
sudo apt update && sudo apt install -y minicom
```

To monitor the serial device:
```bash
minicom -D /dev/ttyACM1
# Ctrl+A, X to quit
```

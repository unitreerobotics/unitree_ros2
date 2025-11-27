# Unitree ROS2 support

[TOC]

## Introduction

[Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2) implements an
easy-to-use robot communication mechanism based on cyclonedds, which enable
developers to achieve robot communication and control. It supports the Unitree
go2/w, B2, and H1 robots.

[DDS](https://www.dds-foundation.org/what-is-dds-3) is also used by ROS2 as a
communication background. Therefore, the underlying layers of the Unitree
go2/w, B2, and H1 robots can be made compatible and ROS2 messages can be used
for communication and control without directly wrapping the Unitree SDK2.

This repository implements a compatibility layer for such purpose.

## System requirements

Tested systems and ROS2 distros:

| systems | ROS2 distro |
|--|--|
|Ubuntu 20.04|foxy|
|Ubuntu 22.04|humble (recommended)|
|Ubuntu 24.04|jazzy|

If you want to use the Docker container environment, you can refer to the
`Dockerfile` related content in the `.devcontainer` folder. Another options is
to use the Dev Container feature of VSCode or other IDEs to create a
development environment, or use Github's codespace to quickly create one. If
you encounter problems while compiling, refer to the compilation scripts in 
`.github/workflows` for guidance.

## Installation

### Installing ROS2 foxy

(If you need another version of ROS2, replace "foxy" with the current ROS2 
version name in the corresponding places).

Install ROS2 foxy following the instructions on
https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html.

### Installing the unitree_ros2 package

#### 1. Clone the repository

Once ROS2 is installed, open a terminal and clone the repository:

```bash
git clone https://github.com/unitreerobotics/unitree_ros2
```

containing the following ROS2 workspaces:

- **cyclonedds_ws**: The workspace of Unitree ROS2 package. The msg for Unitree
  robot are supplied in the subfolder cyclonedds_ws/unitree/unitree_go and
  cyclonedds_ ws/unitree/unitree_api.
- **example_ws**: The workspace of a few examples.

#### 2. Install dependencies

```bash
sudo apt install build-dep ros-foxy-cyclonedds
sudo apt install ros-foxy-rosidl-generator-dds-idl libyaml-cpp-dev
```

#### 3. Compile cyclonedds

(If using humble+, this step can be skipped - just install 
`ros-$ROS_DISTRO-cyclonedds ros-$ROS_DISTRO-rmw-cyclonedds-cpp` with `apt`)

The cyclonedds version installed on the Unitree robot's embedded computers is
0.10.2. To communicate with a Unitree robot using ROS2, it is necessary to
[change the default DDS
implementation](https://docs.ros.org/en/foxy/Concepts/About-Different-Middleware-Vendors.html)

Before compiling cyclonedds, please ensure that ros2 environment has **NOT**
been sourced when starting the terminal. -- it may cause errors upon
compilation -- if `source/opt/ros/foxy/setup.bash` has been added to the
~/.bashrc file when installing ROS2, it needs to be commented out:

```bash
sudo apt install gedit
sudo gedit ~/.bashrc
``` 

```bash
# source /opt/ros/foxy/setup.bash 
```

Then compile cyclonedds:

```bash
cd ~/unitree_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b foxy
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
cd ..

# If the build process failed, try running `export LD_LIBRARY_PATH=/opt/ros/foxy/lib` first.
colcon build --symlink-install --packages-select cyclonedds
```

#### 4. Compile the unitree_ros2 packages

After compiling cyclonedds, a few ROS2 dependencies are required for compiling
the `unitree_api`, `unitree_go` and `unitree_hg` packages on the
`cyclonedds_ws` workspace, so make sure to source the environment of ROS2
before compiling the packages:

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
```

## Robot connection

### 1. Network configuration

You need to setup a static network connection. Connect the Unitree 
robot and the computer using an ethernet cable. Use `ifconfig` or `ip link` to 
view check the network interface the robot is connected to.

Then configure a static IPv4 connection on that interface - change the IPv4 mode 
to manual and set the address and network mask accordingly to the robot model:

| Unitree robot | address/mask      |
| ------------- | ----------------- |
| go2           | 192.168.123.99/24 |
| go2w          | 192.168.123.51/24 |

Finally, setup the ROS2 environment:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
```

And export the `RWM_IMPLEMENTATION` and `CYCLONE_DDS_URI` environment variables:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS>
    <Domain>
        <General>
            <AllowMulticast>spdp</AllowMulticast>
            <Interfaces>
                <NetworkInterface name="enXXXX" priority="default" multicast="default" />
            </Interfaces>
        </General>
    </Domain>
</CycloneDDS>'
```

substituting "enXXXX" with the configured ethernet network interface name.

The `setup.sh` bash script will attempt to configure the first en* interface 
found by `ip link` automatically:

```bash
source ~/unitree_ros2/setup.sh
```

If you don't want to source the bash script every time when a new terminal
opens, you can write the content of bash script into `~/.bashrc`, but pay
attention when there are multiple ROS environments coexisting on your system.

If your computer is not connected to the robot but you still want to use the 
packages on a simulated environment, you can use the local loopback `lo` as 
the network interface:

```bash
source ~/unitree_ros2/setup_local.sh
```

or simply don't specify a network interface:

```bash
source ~/unitree_ros2/setup_default.sh
```

### 2. Testing

After completing the above configuration, it is recommended to restart the 
computer before conducting the test.

Ensure that the robot's network connection is up, open a terminal and input:

```bash
source ~/unitree_ros2/setup.sh
ros2 topic list
```

and you should see an extensive topic list. Input  `ros2 topic echo /lowstate` 
to verify topic messages being published.

### 3. Examples

The source code of examples locates at `/example_ws/src/src`.

| package/directory | description | robot model
|--|--|--|
| common                                 | Common functions                   | all              |
| g1/lowlevel/g1_low_level_example       | Low level control                  | G1               |
| h1-2/lowlevel/low_level_ctrl_hg        | Low level control                  | H1-2             |
| low_level_ctrl                         | Low level control                  | go2/w and B2     |
| read_low_state                         | Read the low state                 | go2/w and B2     |
| read_low_state_hg                      | Read the low state                 | G1 and H1-2      |
| read_motion_state                      | Read the motion state              | go2/w and B2     |
| read_wireless_controller               | Read the wireless controller state | G1, go2/w and B2 |
| record_bag                             | ros2 bag recording example         | all              |
| go2/go2_sport_client                   | High level sportmode control       | go2/w            |
| go2/go2_stand_example                  | Robot stand example                | go2/w            |
| go2/go2_robot_state_client             | Robot state example                | go2/w            |

Open a terminal and input:

```bash
source ~/unitree_ros2/setup.sh
cd ~/unitree_ros2/example_ws
colcon build --symlink-install
```

to compile the examples. Then run:

```bash
./install/unitree_ros2_example/bin/read_motion_state 
```

and you should see the robot status information output on the terminal.

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
sudo apt install build-dep ros-foxy-rmw-cyclonedds-cpp
sudo apt install ros-foxy-rosidl-generator-dds-idl libyaml-cpp-dev
```

#### 3. Compile cyclonedds (If using humble+, this step can be skipped)

(If using humble+, this step can be skipped - just install 
`ros-$ROS_DISTRO-rwm-cyclonedds-cpp` with `apt`)

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

#### 4. Compile unitree_api, unitree_go and unitree_hg packages

After compiling cyclonedds, a few ROS2 dependencies are required for compiling
the `unitree_api`, `unitree_go` and `unitree_hg` packages on the
`cyclonedds_ws` workspace, therefore source the environment of ROS2 before
compiling the packages:

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
```

## Unitree robot connection

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

The `setup.sh' bash script will attempt to configure the first en* interface 
found by `ip link' automatically:

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

You should see the following topics:

```
api/assistant_recorder/request
/api/assistant_recorder/response
/api/audiohub/request
/api/audiohub/response
/api/bashrunner/request
/api/bashrunner/response
/api/config/request
/api/config/response
/api/fourg_agent/request
/api/fourg_agent/response
/api/gas_sensor/request
/api/gas_sensor/response
/api/gpt/request
/api/gpt/response
/api/motion_switcher/request
/api/motion_switcher/response
/api/obstacles_avoid/request
/api/obstacles_avoid/response
/api/pet/request
/api/pet/response
/api/programming_actuator/request
/api/programming_actuator/response
/api/robot_state/request
/api/robot_state/response
/api/sport/request
/api/sport/response
/api/sport_lease/request
/api/sport_lease/response
/api/uwbswitch/request
/api/uwbswitch/response
/api/videohub/request
/api/videohub/response
/api/vui/request
/api/vui/response
/arm_Command
/arm_Feedback
/audiohub/player/state
/audioreceiver
/audiosender
/config_change_status
/frontvideostream
/gas_sensor
/gnss
/gpt_cmd
/gptflowfeedback
/lf/lowstate
/lf/sportmodestate
/lio_sam_ros2/mapping/odometry
/lowcmd
/lowstate
/multiplestate
/parameter_events
/pctoimage_local
/pet/flowfeedback
/programming_actuator/command
/programming_actuator/feedback
/public_network_status
/qt_add_edge
/qt_add_node
/qt_command
/qt_notice
/query_result_edge
/query_result_node
/rosout
/rtc/state
/rtc_status
/selftest
/servicestate
/servicestateactivate
/sportmodestate
/uslam/client_command
/uslam/frontend/cloud_world_ds
/uslam/frontend/odom
/uslam/localization/cloud_world
/uslam/localization/odom
/uslam/navigation/global_path
/uslam/server_log
/utlidar/client_cmd
/utlidar/cloud
/utlidar/cloud_base
/utlidar/cloud_deskewed
/utlidar/grid_map
/utlidar/height_map
/utlidar/height_map_array
/utlidar/imu
/utlidar/lidar_state
/utlidar/mapping_cmd
/utlidar/range_info
/utlidar/range_map
/utlidar/robot_odom
/utlidar/robot_pose
/utlidar/server_log
/utlidar/switch
/utlidar/voxel_map
/utlidar/voxel_map_compressed
/uwbstate
/uwbswitch
/videohub/inner
/webrtcreq
/webrtcres
/wirelesscontroller
/wirelesscontroller_unprocessed
/xfk_webrtcreq
/xfk_webrtcres
```

Input `ros2 topic echo /lowstate --once`，you can see the topic data:

```
stamp:
  sec: 0
  nanosec: 0
error_code: 0
imu_state:
  quaternion:
  - 0.9934075474739075
  - -0.0005416878848336637
  - 0.004365340806543827
  - 0.11455302685499191
  gyroscope:
  - -0.009587379172444344
  - -0.004261057358235121
  - -0.004261057358235121
  accelerometer:
  - -0.07541735470294952
  - 0.038307227194309235
  - 9.518148422241211
  rpy:
  - -7.611058390466496e-05
  - 0.008797342889010906
  - 0.22961203753948212
  temperature: 79
mode: 1
progress: 0.0
gait_type: 1
foot_raise_height: 0.0
position:
- 0.0
- 0.0
- 0.0
body_height: 0.0
velocity:
- 0.0
- 0.0
- 0.0
yaw_speed: 0.0
range_obstacle:
- 0.0
- 0.0
- 0.0
- 0.0
foot_force:
- 0
- 0
- 0
- 0
foot_position_body:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
foot_speed_body:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
---
```
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

You should see the robot status information output on the terminal:

```bash
[INFO] [1697525196.266174885] [motion_state_suber]: Position -- x: 0.567083; y: 0.213920; z: 0.052338; body height: 0.320000
[INFO] [1697525196.266230044] [motion_state_suber]: Velocity -- vx: -0.008966; vy: -0.001431; vz: -0.019455; yaw: -0.002131
[INFO] [1697525196.266282725] [motion_state_suber]: Foot position and velcity relative to body -- num: 0; x: 0.204149; y: -0.145194; z: -0.067804, vx: 0.002683; vy: 0.003745; vz: -0.010052
[INFO] [1697525196.266339057] [motion_state_suber]: Foot position and velcity relative to body -- num: 1; x: 0.204200; y: 0.145049; z: -0.068205, vx: -0.001954; vy: -0.003442; vz: -0.004828
[INFO] [1697525196.266392028] [motion_state_suber]: Foot position and velcity relative to body -- num: 2; x: -0.183385; y: -0.159294; z: -0.039468, vx: -0.000739; vy: -0.002028; vz: -0.004532
[INFO] [1697525196.266442766] [motion_state_suber]: Foot position and velcity relative to body -- num: 3; x: -0.182412; y: 0.159754; z: -0.039045, vx: -0.002803; vy: -0.001381; vz: -0.004794
[INFO] [1697525196.316189064] [motion_state_suber]: Gait state -- gait type: 1; raise height: 0.090000
```

## Usage

### State acquisition

#### 1. Sportmode state

The sportmode state message includes position, velocity, foot position, and
other motion state information. To access it, subscribe to the `/sportmodestate` 
or the `/lf/sportmodestate` topic, where "lf" indicates low frequency.

The .msg file is defined as:

```C++
TimeSpec        stamp       // time stamp
uint32          error_code  // error code
IMUState        imu_state   // imu state
uint8           mode        // sportmode
/*
Sportmode
0.  idle, default stand
1.  balance stand
2.  pose
3.  locomotion
4.  reserve
5.  lie sown
6.  joint lock
7.  damping
8.  recovery stand
9.  reserve
10. sit
11. front flip
12. front jump
13. front pounce
*/
float32         progress    // is a dance action being executed?：0. dance false; 1. dance true
uint8           gait_type   // gait type
/*
Gait type
0.  idle  
1.  trot  
2.  run  
3.  climb up stairs  
4.  climb down stairs  
9.  adjust
*/
float32         foot_raise_height 
float32[3]      position 
float32         body_height
float32[3]      velocity 
float32         yaw_speed
float32[4]      range_obstacle
int16[4]        foot_force 
float32[12]     foot_position_body  // foot positions in body frame
float32[12]     foot_speed_body     // foot velocities in body frame
```

For more details, see https://support.unitree.com/home/en/developer/sports_services

To run a complete example:

```bash
./install/unitree_ros2_example/bin/read_motion_state
```

### 2. Low-level state

The low-level state message includes motors states, power information, and
other low-level information. The low-level state can be obtained by
subscribing to the `/lowstate` or the `/lf/lowstate` topic.

The .msg file is defined as:

```C++
uint8[2]        head
uint8           level_flag
uint8           frame_reserve
uint32[2]       sn
uint32[2]       version
uint16          bandwidth
IMUState        imu_state           // imu state
MotorState[20]  motor_state         // motor state
BmsState        bms_state
int16[4]        foot_force 
int16[4]        foot_force_est
uint32          tick
uint8[40]       wireless_remote
uint8           bit_flag
float32         adc_reel
int8            temperature_ntc1
int8            temperature_ntc2
float32         power_v 
float32         power_a 
uint16[4]       fan_frequency 
uint32          reserve
uint32          crc
```

where the MotorState struct is defined as:

```C++
uint8           mode                // mode, 0x01 for control
float32         q                   // joint angle
float32         dq                  // joint velocity
float32         ddq                 // joint acceleration
float32         tau_est             // estimated torque
float32         q_raw               // raw data of q
float32         dq_raw              // raw data of dq
float32         ddq_raw             // raw data of dq
int8            temperature 
uint32          lost
uint32[2]       reserve
```

For more details, see https://support.unitree.com/home/en/developer/Basic_services

To run a complete example:

```bash
./install/unitree_ros2_example/bin/read_low_state
```

Complete examples is in example/src/read_low_state.cpp. 

### 3. Wireless controller

The wireless controller state can be obtained by subscribing to the 
`/wirelesscontroller` topic. You can directly write a messsage to the topic to
control the robot as if pressing the robot's own wireless controller.

The .msg file is defiened as:

```C++
float32 lx      // left joystick x, range [-1.0 ~ 1.0]
float32 ly      // left joystick y, range [-1.0 ~ 1.0]
float32 rx      // right joystick x, range [-1.0 ~ 1.0]
float32 ry      // right joystick y, range [-1.0 ~ 1.0]
uint16  keys    // pressed keys
```

For more details, see https://support.unitree.com/home/en/developer/Get_remote_control_status

To run a complete example:

```bash
./install/unitree_ros2_example/bin/read_wireless_controller
```

## Robot control

### 1. Sportmode 

Sportmode control can be achieved by sending a `unitree_api::msg::Request`
message to the `/api/sport/request` topic.

The Request message can be generated by the SportClient class in 
`/example/src/common/ros2_sport_client.cpp`. For example:

```C++
// Create a ros2 publisher
rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

SportClient sport_req;                  // Sportclient object
unitree_api::msg::Request req;          // Sportmode request msg
sport_req.Euler(req, r, p, y);          // Sportmode request msg from Sportclient

req_puber->publish(req);                // Publish request msg
```

For details on SportClient, see https://support.unitree.com/home/en/developer/sports_services

To run a complete example:

```bash
./install/unitree_ros2_example/bin/sport_mode_ctrl
```

Ater 1 second, the robot will walk back and forth in the x direction.

### 2. Motor control

The torque, position and velocity control of each motor can be controlled by
subscribing "/lowcmd" topic and sending `unitree_go::msg::LowCmd` messages. 

The .msg file is defined as:

```C++
uint8[2]        head
uint8           level_flag
uint8           frame_reserve
uint32[2]       sn
uint32[2]       version
uint16          bandwidth
MotorCmd[20]    motor_cmd           // motor command
BmsCmd          bms_cmd
uint8[40]       wireless_remote
uint8[12]       led
uint8[2]        fan
uint8           gpio
uint32          reserve
uint32          crc
```

where the MotorCmd struct is defined as:

```C++
uint8 mode;     // mode (foc mode -> 0x01，stop mode -> 0x00)
float q;        // target position (rad)
float dq;       // target velocity (rad/s)
float tau;      // target torque (N.M)
float kp;       // motor's PD controller's kp 
float kd;       // motor's PD controller's kd
unsigned long   reserve[3]; 
```

For more details, see https://support.unitree.com/home/en/developer/Basic_services

To run a complete example:

```bash
./install/unitree_ros2_example/bin/sport_mode_ctrl
```

The hip and calf motors of the RL leg will rotate to the corresponding joint
angle.

## Rviz

We can also use rviz to visualize the Unitree robot data.

First, list all topics with

```bash
ros2 topic list
```

so we can verify that the integrated lidar point cloud information is published
on the

```bash
utlidar/cloud
```

topic. Then, print its `frame_id`

```
ros2 topic echo --no-arr /utlidar/cloud
```

and you should get `utlidar_lidar`.

Finally, run rviz：

```
ros2 run rviz2 rviz2
```

and add the `/utlidar/cloud` in rviz2 and modify the `Fixed frame` to
`utlidar_lidar`.


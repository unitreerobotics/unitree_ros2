#!/bin/bash
echo "Setup unitree ros2 environment"

# Get first en* interface name
IFACE_NAME=$(ip -br l | awk '$1 ~ "en" NR==1 { print $1 }' | awk 'NR==1')

source /opt/ros/$ROS_DISTRO/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Change to the network interface in which the unitree robot is connected
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="${IFACE_NAME}" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>"

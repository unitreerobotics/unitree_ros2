#!/bin/bash
echo "Setup unitree ros2 environment"
source /opt/ros/$ROS_DISTRO/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Modify INTERFACE_NAME to the network interface in which the unitree robot is connected
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="INTERFACE_NAME" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'

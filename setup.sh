#!/bin/bash
echo "Setup unitree ros2 environment"

# Grab first en* interface name by default
IFACE_NAME=${1:-$(ip -br l | awk '$1 ~ "en" { print $1 }' | head -n 1)}

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Change to the network interface in which the unitree robot is connected
export CYCLONEDDS_URI='<CycloneDDS>
    <Domain>
        <General>
            <Interfaces>
                <NetworkInterface name="'$IFACE_NAME'" priority="'default'" multicast="'default'"/>
            </Interfaces>
        </General>
    </Domain>
</CycloneDDS>'

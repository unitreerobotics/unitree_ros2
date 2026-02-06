#!/bin/bash
echo "Setup unitree ros2 simulation environment"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS>
    <Domain>
        <General>
            <Interfaces>
                <NetworkInterface name="lo" priority="default" multicast="default" />
            </Interfaces>
        </General>
    </Domain>
</CycloneDDS>'

#!/bin/bash
# Build an isolated firmware-era DDS peer and test it against the current ROS RMW.
set -eo pipefail
if [[ -z "${ROS_DISTRO:-}" || -z "${1:-}" ]]; then
  echo "Usage: source ROS and the Unitree install, then $0 SCRATCH_DIRECTORY" >&2
  exit 2
fi
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$1"
scratch="$(cd "$1" && pwd)"
touch "$scratch/COLCON_IGNORE"
revision=9995905bce6c4cf9f740d6438bbf7fcfd1c83dfd
if [[ ! -d "$scratch/source/.git" ]]; then
  git clone --branch 0.10.2 --depth 1 https://github.com/eclipse-cyclonedds/cyclonedds.git "$scratch/source"
fi
test "$(git -C "$scratch/source" rev-parse HEAD)" = "$revision"
# This prefix is passed only to the standalone peer build, never the ROS build.
env -i HOME="$HOME" PATH=/usr/bin:/bin LANG=C.UTF-8 \
  cmake -S "$scratch/source" -B "$scratch/build" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$scratch/install" \
    -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DENABLE_SSL=NO -DENABLE_SECURITY=NO -DENABLE_SHM=OFF
env -i HOME="$HOME" PATH=/usr/bin:/bin LANG=C.UTF-8 \
  cmake --build "$scratch/build" --parallel 2
env -i HOME="$HOME" PATH=/usr/bin:/bin LANG=C.UTF-8 \
  cmake --install "$scratch/build"
cmake -S "$root/test/unitree_ros2_test/native" -B "$scratch/peer" \
  -DCycloneDDS_DIR="$scratch/install/lib/cmake/CycloneDDS"
LD_LIBRARY_PATH="$scratch/install/lib" cmake --build "$scratch/peer" --parallel 2
# The ROS-side API client driver comes from the normal workspace build.
client="${UNITREE_API_CLIENT_PEER_TEST:-$root/build/unitree_ros2_test/api_client_peer_test}"
b2_client="${UNITREE_B2_API_CLIENT_PEER_TEST:-$root/build/unitree_ros2_test/b2_api_client_peer_test}"
if [[ ! -x "$client" || ! -x "$b2_client" ]]; then
  echo "Build unitree_ros2_test with tests first, or set UNITREE_API_CLIENT_PEER_TEST" \
    "and UNITREE_B2_API_CLIENT_PEER_TEST" >&2
  exit 2
fi
UNITREE_DDS_PEER="$scratch/peer/dds_peer" \
UNITREE_DDS_API_SERVER="$scratch/peer/dds_api_server" \
UNITREE_API_CLIENT_PEER_TEST="$client" \
UNITREE_B2_API_CLIENT_PEER_TEST="$b2_client" \
UNITREE_DDS_PEER_LIBRARY_PATH="$scratch/install/lib" \
  python3 -m pytest "$root/test/unitree_ros2_test/test_native_dds.py" \
    "$root/test/unitree_ros2_test/test_api_client_peer.py" \
    --junitxml="$scratch/native-dds-0102.xml" -v

#!/bin/bash
# Run the API client/plain DDS server tests with added loopback latency (tc netem),
# modelling a slow robot link where DDS discovery outlasts the clients' startup
# wait. This changes the qdisc of "lo" for every process on the host while it runs:
# use it only in a container or a disposable CI VM.
# Usage: source ROS and the workspace, then
#   UNITREE_ALLOW_LO_NETEM=1 scripts/test_api_latency.sh [DELAY] [JUNIT_XML]
# Exit 3 means netem is unavailable on this host (not a test failure).
set -eo pipefail
delay="${1:-250ms}"
junit="${2:-}"
if [[ "${UNITREE_ALLOW_LO_NETEM:-}" != 1 ]]; then
  echo "Refusing to change lo latency system-wide; set UNITREE_ALLOW_LO_NETEM=1." >&2
  exit 2
fi
if [[ ! "$delay" =~ ^[0-9]+ms$ ]]; then
  echo "DELAY must look like 250ms" >&2
  exit 2
fi
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build="${UNITREE_TEST_BUILD:-$root/build/unitree_ros2_test}"
sudo=()
if [[ $EUID -ne 0 ]]; then sudo=(sudo -n); fi
if ! "${sudo[@]}" tc qdisc add dev lo root netem delay "$delay" 2>/dev/null; then
  "${sudo[@]}" modprobe sch_netem 2>/dev/null || true
  if ! "${sudo[@]}" tc qdisc add dev lo root netem delay "$delay"; then
    echo "tc netem unavailable on this host" >&2
    exit 3
  fi
fi
trap '"${sudo[@]}" tc qdisc del dev lo root' EXIT
UNITREE_DDS_API_SERVER="$build/dds_api_server" \
UNITREE_API_CLIENT_PEER_TEST="$build/api_client_peer_test" \
UNITREE_B2_API_CLIENT_PEER_TEST="$build/b2_api_client_peer_test" \
  python3 -m pytest "$root/test/unitree_ros2_test/test_api_client_peer.py" -v \
    ${junit:+--junitxml="$junit"}

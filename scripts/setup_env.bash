# Shared implementation for the sourceable setup scripts.
_unitree_setup() {
  local mode="$1" interface="$2" distro="$3"
  local root prefix workspace variable value other candidate
  local -a installed=()
  root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)" || return 1
  distro="${distro:-${ROS_DISTRO:-}}"
  if [[ -z "$distro" && -z "${UNITREE_ROS_PREFIX:-}" ]]; then
    # The legacy scripts sourced a fixed distribution without arguments.
    # Keep argument-free sourcing working when exactly one is installed.
    for candidate in /opt/ros/*/setup.bash; do
      [[ -f "$candidate" ]] && installed+=("$(basename "$(dirname "$candidate")")")
    done
    if [[ ${#installed[@]} -eq 1 ]]; then
      distro="${installed[0]}"
    fi
  fi
  if [[ -z "$distro" ]]; then
    echo "Select a ROS distribution: source setup_local.sh jazzy (or source ROS first)." >&2
    if [[ ${#installed[@]} -gt 1 ]]; then
      echo "Installed distributions: ${installed[*]}" >&2
    fi
    return 1
  fi
  if [[ ! "$distro" =~ ^[a-z]+$ ]]; then
    echo "Invalid ROS distribution name: $distro" >&2
    return 1
  fi
  if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != "$distro" ]]; then
    echo "A different ROS distribution is already sourced. Start a clean shell." >&2
    return 1
  fi
  for variable in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH; do
    value="${!variable}"
    for other in foxy galactic humble iron jazzy kilted rolling; do
      if [[ "$other" != "$distro" && ( "$value" == *"/opt/ros/$other/"* || ":$value:" == *"/opt/ros/$other:"* ) ]]; then
        echo "$variable contains another ROS distribution. Start a clean shell." >&2
        return 1
      fi
    done
  done
  prefix="${UNITREE_ROS_PREFIX:-/opt/ros/$distro}"
  if [[ -n "${UNITREE_ROS_WS:-}" ]]; then
    workspace="$UNITREE_ROS_WS"
  elif [[ -f "$root/cyclonedds_ws/install/local_setup.bash" ]]; then
    # Legacy layout: interfaces (and, for Foxy, Cyclone DDS) in cyclonedds_ws.
    workspace="$root/cyclonedds_ws"
  else
    # Single repository-root workspace, as built by docs/jazzy.md.
    workspace="$root"
  fi
  if [[ ! -f "$prefix/setup.bash" || ! -f "$workspace/install/local_setup.bash" ]]; then
    echo "ROS setup or Unitree install missing. Build first; set UNITREE_ROS_WS for an external workspace." >&2
    return 1
  fi
  if [[ "$mode" == network ]]; then
    interface="${interface:-${UNITREE_NETWORK_INTERFACE:-}}"
    if [[ ! "$interface" =~ ^[a-zA-Z0-9_.:-]+$ || ! -d "/sys/class/net/$interface" ]]; then
      echo "Specify an existing robot-facing interface: source setup.sh INTERFACE [DISTRO]" \
        "(or export UNITREE_NETWORK_INTERFACE)." >&2
      return 1
    fi
  fi
  source "$prefix/setup.bash" || return 1
  source "$workspace/install/local_setup.bash" || return 1
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  case "$mode" in
    local)
      unset ROS_LOCALHOST_ONLY
      export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-173}"
      export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" multicast="true"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>32</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
      ;;
    network)
      unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE
      export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$interface\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
      ;;
    default)
      unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE CYCLONEDDS_URI
      ;;
  esac
  echo "Setup unitree ros2 environment: $distro, $mode, $workspace"
}

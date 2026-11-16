#!/bin/bash
# Usage: source setup.sh INTERFACE [DISTRO]
source "$(dirname "${BASH_SOURCE[0]}")/scripts/setup_env.bash" || return 1
_unitree_setup network "${1:-}" "${2:-}"

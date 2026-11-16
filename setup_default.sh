#!/bin/bash
# Usage: source setup_default.sh [DISTRO]
source "$(dirname "${BASH_SOURCE[0]}")/scripts/setup_env.bash" || return 1
_unitree_setup default "" "${1:-}"

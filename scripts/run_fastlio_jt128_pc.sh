#!/usr/bin/env bash
set -euo pipefail

workspace_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
cyclonedds_config="$workspace_dir/config/cyclonedds_unitree_a2.xml"
ros2_bin=${ROS2_BIN:-ros2}

set +u
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$cyclonedds_config"
export ROS_LOG_DIR="$workspace_dir/log/fast_lio_runtime"

exec "$ros2_bin" launch fast_lio mapping_jt128.launch.py "$@"

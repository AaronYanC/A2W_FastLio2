#!/usr/bin/env bash
set -euo pipefail

repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
# shellcheck source=scripts/lib/a2w_common.sh
source "$repository_root/scripts/lib/a2w_common.sh"

ros2_bin=${ROS2_BIN:-ros2}

a2w_source_ros_environment "$repository_root"
a2w_prepare_cyclonedds "$repository_root"

status=0
"$ros2_bin" topic info /lidar_points --verbose || status=1
"$ros2_bin" topic info /lidar_imu --verbose || status=1

if ((status != 0)); then
    a2w_fail "one or more JT128 input topics are unavailable"
    exit "$status"
fi

printf 'A2W Fast-LIO2 input topics are visible.\n'

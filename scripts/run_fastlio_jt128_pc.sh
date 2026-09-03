#!/usr/bin/env bash
set -euo pipefail

repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
# shellcheck source=scripts/lib/a2w_common.sh
source "$repository_root/scripts/lib/a2w_common.sh"

ros2_bin=${ROS2_BIN:-ros2}
launch_arguments=("$@")
save_map_argument_found=false

for argument in "${launch_arguments[@]}"; do
    if [[ $argument == save_map:=* ]]; then
        save_map_argument_found=true
        break
    fi
done
if [[ $save_map_argument_found == false ]]; then
    launch_arguments=("save_map:=false" "${launch_arguments[@]}")
fi

a2w_source_ros_environment "$repository_root"
a2w_prepare_cyclonedds "$repository_root"

exec "$ros2_bin" launch a2w_fastlio2_bringup jt128_mapping.launch.py \
    "${launch_arguments[@]}"

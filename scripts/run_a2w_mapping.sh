#!/usr/bin/env bash
set -euo pipefail

repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
pc_launcher=${A2W_PC_LAUNCHER:-"$repository_root/scripts/run_fastlio_jt128_pc.sh"}
bundle_root=${A2W_BUNDLE_ROOT:-"$repository_root/maps"}
launch_arguments=("$@")
bundle_root_found=false
for argument in "${launch_arguments[@]}"; do
    if [[ $argument == bundle_root:=* ]]; then
        bundle_root_found=true
        break
    fi
done
if [[ $bundle_root_found == false ]]; then
    launch_arguments=("bundle_root:=$bundle_root" "${launch_arguments[@]}")
fi

export A2W_BRINGUP_LAUNCH=mapping.launch.py
export A2W_SAVE_ARGUMENT_NAME=
exec "$pc_launcher" "${launch_arguments[@]}"

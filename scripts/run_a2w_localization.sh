#!/usr/bin/env bash
set -euo pipefail

repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
pc_launcher=${A2W_PC_LAUNCHER:-"$repository_root/scripts/run_fastlio_jt128_pc.sh"}

if [[ $# -lt 1 || $1 == *:=* ]]; then
    printf 'Usage: %s MAP_BUNDLE_PATH [launch_argument:=value ...]\n' "$0" >&2
    exit 2
fi

map_bundle_path=$(realpath -e -- "$1")
shift
if [[ ! -d $map_bundle_path || ! -f $map_bundle_path/manifest.sha256 ]]; then
    printf 'A2W Fast-LIO2: invalid Map Bundle directory: %s\n' "$map_bundle_path" >&2
    exit 1
fi

export A2W_BRINGUP_LAUNCH=localization.launch.py
export A2W_SAVE_ARGUMENT_NAME=
exec "$pc_launcher" "map_bundle_path:=$map_bundle_path" "$@"

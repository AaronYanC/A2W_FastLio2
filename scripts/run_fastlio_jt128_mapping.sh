#!/usr/bin/env bash
set -euo pipefail

repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
pc_launcher=${A2W_PC_LAUNCHER:-"$repository_root/scripts/run_fastlio_jt128_pc.sh"}
map_file=${A2W_MAP_FILE:-"$repository_root/maps/jt128_map.pcd"}

mkdir -p "$(dirname "$map_file")"

exec "$pc_launcher" "save_map:=true" "map_file:=$map_file" "$@"

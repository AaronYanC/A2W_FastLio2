#!/usr/bin/env bash
set -euo pipefail

workspace_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
pc_launcher=${FAST_LIO_PC_LAUNCHER:-"$workspace_dir/scripts/run_fastlio_jt128_pc.sh"}
config_file="$workspace_dir/config/jt128_mapping_save.yaml"

mkdir -p "$workspace_dir/maps"

exec "$pc_launcher" "config_file:=$config_file" "$@"

#!/usr/bin/env bash
set -euo pipefail

repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
pc_launcher=${A2W_PC_LAUNCHER:-"$repository_root/scripts/run_fastlio_jt128_pc.sh"}

export A2W_BRINGUP_LAUNCH=mapping_stage1.launch.py
export A2W_SAVE_ARGUMENT_NAME=save_frontend_pcd

exec "$pc_launcher" "$@"

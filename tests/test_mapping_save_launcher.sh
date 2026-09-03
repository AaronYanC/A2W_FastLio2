#!/usr/bin/env bash
set -euo pipefail

workspace_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
config_file="$workspace_dir/config/jt128_mapping_save.yaml"
launcher="$workspace_dir/scripts/run_fastlio_jt128_mapping.sh"

fail() {
    printf 'FAIL: %s\n' "$1" >&2
    exit 1
}

[[ -f "$config_file" ]] || fail "missing map-save config: $config_file"
[[ -x "$launcher" ]] || fail "missing executable map-save launcher: $launcher"

/usr/bin/python3 - "$config_file" "$workspace_dir" <<'PY'
import os
import sys
import yaml

config_path, workspace = sys.argv[1:]
with open(config_path, encoding="utf-8") as stream:
    params = yaml.safe_load(stream)["/**"]["ros__parameters"]

assert params["pcd_save"]["pcd_save_en"] is True
assert params["pcd_save"]["interval"] == -1
assert params["pcd_save"]["leaf_size"] == 0.1
assert params["map_file_path"] == os.path.join(workspace, "maps", "jt128_map.pcd")
assert params["common"]["lid_topic"] == "/lidar_points"
assert params["common"]["imu_topic"] == "/lidar_imu"
assert params["preprocess"]["lidar_type"] == 2
assert params["preprocess"]["scan_line"] == 128
PY

temp_dir=$(mktemp -d)
trap 'rm -rf "$temp_dir"' EXIT
capture_file="$temp_dir/capture"
fake_pc_launcher="$temp_dir/fake_pc_launcher"

cat >"$fake_pc_launcher" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
printf '%s\n' "$@" >"$FAST_LIO_TEST_CAPTURE"
EOF
chmod +x "$fake_pc_launcher"

FAST_LIO_TEST_CAPTURE="$capture_file" \
FAST_LIO_PC_LAUNCHER="$fake_pc_launcher" \
    "$launcher" rviz:=false

grep -Fx "config_file:=$config_file" "$capture_file" >/dev/null || \
    fail "map-save launcher did not select the save config"
grep -Fx 'rviz:=false' "$capture_file" >/dev/null || \
    fail "map-save launcher did not forward launch arguments"

printf 'PASS: JT128 map-save launcher contract\n'

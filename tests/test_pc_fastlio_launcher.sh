#!/usr/bin/env bash
set -euo pipefail

workspace_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
config_file="$workspace_dir/config/cyclonedds_unitree_a2.xml"
launcher="$workspace_dir/scripts/run_fastlio_jt128_pc.sh"

fail() {
    printf 'FAIL: %s\n' "$1" >&2
    exit 1
}

[[ -f "$config_file" ]] || fail "missing CycloneDDS config: $config_file"
[[ -x "$launcher" ]] || fail "missing executable launcher: $launcher"

configured_address=$(/usr/bin/python3 -c '
import sys
import xml.etree.ElementTree as ET

root = ET.parse(sys.argv[1]).getroot()
interface = root.find(".//{*}NetworkInterface")
if interface is None:
    raise SystemExit("NetworkInterface is missing")
print(interface.attrib.get("address", ""))
' "$config_file")
[[ "$configured_address" == "192.168.123.100" ]] || \
    fail "CycloneDDS interface address is '$configured_address'"

temp_dir=$(mktemp -d)
trap 'rm -rf "$temp_dir"' EXIT
capture_file="$temp_dir/capture"
fake_ros2="$temp_dir/ros2"

cat >"$fake_ros2" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
{
    printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-}"
    printf 'CYCLONEDDS_URI=%s\n' "${CYCLONEDDS_URI:-}"
    printf 'ROS_LOG_DIR=%s\n' "${ROS_LOG_DIR:-}"
    printf 'ARGS='
    printf '%q ' "$@"
    printf '\n'
} >"$FAST_LIO_TEST_CAPTURE"
EOF
chmod +x "$fake_ros2"

FAST_LIO_TEST_CAPTURE="$capture_file" ROS2_BIN="$fake_ros2" \
    "$launcher" rviz:=false

grep -Fx 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' "$capture_file" >/dev/null || \
    fail "launcher did not select CycloneDDS"
grep -Fx "CYCLONEDDS_URI=file://$config_file" "$capture_file" >/dev/null || \
    fail "launcher did not select the workspace CycloneDDS config"
grep -Fx "ROS_LOG_DIR=$workspace_dir/log/fast_lio_runtime" "$capture_file" >/dev/null || \
    fail "launcher did not keep ROS logs inside the workspace"
grep -Fx 'ARGS=launch fast_lio mapping_jt128.launch.py rviz:=false ' \
    "$capture_file" >/dev/null || fail "launcher arguments are incorrect"

printf 'PASS: PC FAST-LIO2 launcher contract\n'

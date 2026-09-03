#!/usr/bin/env bash
set -euo pipefail

repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
pc_launcher="$repository_root/scripts/run_fastlio_jt128_pc.sh"
mapping_launcher="$repository_root/scripts/run_fastlio_jt128_mapping.sh"

fail() {
    printf 'FAIL: %s\n' "$1" >&2
    exit 1
}

temp_dir=$(mktemp -d)
trap 'rm -rf "$temp_dir"' EXIT

ros_setup="$temp_dir/ros_setup.bash"
install_setup="$temp_dir/install_setup.bash"
fake_ros2="$temp_dir/ros2"
pc_capture="$temp_dir/pc_capture"
mapping_capture="$temp_dir/mapping_capture"
runtime_dir="$temp_dir/runtime"
map_file="$repository_root/maps/jt128_map.pcd"

: >"$ros_setup"
: >"$install_setup"

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
} >"$A2W_TEST_CAPTURE"
EOF
chmod +x "$fake_ros2"

(
    cd /tmp
    A2W_TEST_CAPTURE="$pc_capture" \
    A2W_ROS_SETUP="$ros_setup" \
    A2W_INSTALL_SETUP="$install_setup" \
    A2W_PC_IP="192.168.123.77" \
    A2W_NETWORK_INTERFACE="test0" \
    A2W_DDS_PEER="192.168.123.164" \
    A2W_RUNTIME_DIR="$runtime_dir" \
    ROS2_BIN="$fake_ros2" \
        "$pc_launcher" rviz:=false
)

grep -Fx 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' "$pc_capture" >/dev/null || \
    fail "PC launcher did not select CycloneDDS"
grep -Fx "CYCLONEDDS_URI=file://$runtime_dir/cyclonedds.xml" "$pc_capture" >/dev/null || \
    fail "PC launcher did not select its generated runtime DDS config"
grep -Fx "ROS_LOG_DIR=$repository_root/log/fast_lio_runtime" "$pc_capture" >/dev/null || \
    fail "PC launcher did not resolve the repository log directory"
grep -Fx \
    'ARGS=launch a2w_fastlio2_bringup jt128_mapping.launch.py save_map:=false rviz:=false ' \
    "$pc_capture" >/dev/null || fail "PC launcher command is not portable bringup"

/usr/bin/python3 - "$runtime_dir/cyclonedds.xml" <<'PY'
import sys
import xml.etree.ElementTree as ET

root = ET.parse(sys.argv[1]).getroot()
interface = root.find(".//{*}NetworkInterface")
peer = root.find(".//{*}Peer")
assert interface is not None and interface.attrib["address"] == "192.168.123.77"
assert peer is not None and peer.attrib["address"] == "192.168.123.164"
PY

fake_ip="$temp_dir/ip"
tunnel_capture="$temp_dir/tunnel_capture"
cat >"$fake_ip" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
if [[ $* == '-o -4 addr show scope global' ]]; then
    printf '%s\n' \
        '2: enp49s0    inet 192.168.123.100/24 brd 192.168.123.255 scope global enp49s0' \
        '9: FlClash    inet 28.0.0.1/8 scope global FlClash'
elif [[ $* == '-4 route get 192.168.123.164' ]]; then
    printf '%s\n' '192.168.123.164 dev FlClash src 28.0.0.1'
else
    printf 'unexpected fake ip arguments: %s\n' "$*" >&2
    exit 2
fi
EOF
chmod +x "$fake_ip"

(
    cd /tmp
    A2W_TEST_CAPTURE="$tunnel_capture" \
    A2W_ROS_SETUP="$ros_setup" \
    A2W_INSTALL_SETUP="$install_setup" \
    A2W_IP_BIN="$fake_ip" \
    A2W_DDS_PEER="192.168.123.164" \
    A2W_RUNTIME_DIR="$runtime_dir" \
    ROS2_BIN="$fake_ros2" \
        "$pc_launcher" rviz:=false
)

/usr/bin/python3 - "$runtime_dir/cyclonedds.xml" <<'PY'
import sys
import xml.etree.ElementTree as ET

root = ET.parse(sys.argv[1]).getroot()
interface = root.find(".//{*}NetworkInterface")
assert interface is not None and interface.attrib["address"] == "192.168.123.100"
PY

cat >"$temp_dir/fake_pc_launcher" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
printf '%s\n' "$@" >"$A2W_TEST_CAPTURE"
EOF
chmod +x "$temp_dir/fake_pc_launcher"

(
    cd /tmp
    A2W_TEST_CAPTURE="$mapping_capture" \
    A2W_PC_LAUNCHER="$temp_dir/fake_pc_launcher" \
    A2W_MAP_FILE="$map_file" \
        "$mapping_launcher" rviz:=false
)

grep -Fx 'save_map:=true' "$mapping_capture" >/dev/null || \
    fail "mapping launcher did not enable map saving"
grep -Fx "map_file:=$map_file" "$mapping_capture" >/dev/null || \
    fail "mapping launcher did not resolve the map below repository root"
grep -Fx 'rviz:=false' "$mapping_capture" >/dev/null || \
    fail "mapping launcher did not forward launch arguments"

tool_capture="$temp_dir/tool_capture"
fake_git="$temp_dir/git"
fake_rosdep="$temp_dir/rosdep"
fake_colcon="$temp_dir/colcon"
fake_topic_ros2="$temp_dir/topic_ros2"

cat >"$temp_dir/tool_recorder" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
printf '%s PWD=%q ARGS=' "$(basename "$0")" "$PWD" >>"$A2W_TEST_CAPTURE"
printf '%q ' "$@" >>"$A2W_TEST_CAPTURE"
printf '\n' >>"$A2W_TEST_CAPTURE"
EOF
chmod +x "$temp_dir/tool_recorder"
ln -s "$temp_dir/tool_recorder" "$fake_git"
ln -s "$temp_dir/tool_recorder" "$fake_rosdep"
ln -s "$temp_dir/tool_recorder" "$fake_colcon"
ln -s "$temp_dir/tool_recorder" "$fake_topic_ros2"

(
    cd /tmp
    A2W_TEST_CAPTURE="$tool_capture" \
    A2W_ROS_SETUP="$ros_setup" \
    GIT_BIN="$fake_git" \
    ROSDEP_BIN="$fake_rosdep" \
        "$repository_root/scripts/bootstrap.sh"
)

grep -F "git PWD=/tmp ARGS=-C $repository_root submodule update --init --recursive " \
    "$tool_capture" >/dev/null || fail "bootstrap did not initialize recursive submodules"
grep -F "rosdep PWD=/tmp ARGS=install --from-paths $repository_root/src --ignore-src -r -y " \
    "$tool_capture" >/dev/null || fail "bootstrap did not install workspace dependencies"

: >"$tool_capture"
(
    cd /tmp
    A2W_TEST_CAPTURE="$tool_capture" \
    A2W_ROS_SETUP="$ros_setup" \
    GIT_BIN="$fake_git" \
    COLCON_BIN="$fake_colcon" \
        "$repository_root/scripts/build.sh"
)

grep -F "git PWD=/tmp ARGS=-C $repository_root submodule update --init --recursive " \
    "$tool_capture" >/dev/null || fail "build did not initialize recursive submodules"
grep -F \
    "colcon PWD=$repository_root ARGS=build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 " \
    "$tool_capture" >/dev/null || \
    fail "build did not pin both modern and legacy ROS Python CMake variables"

: >"$tool_capture"
(
    cd /tmp
    A2W_TEST_CAPTURE="$tool_capture" \
    A2W_ROS_SETUP="$ros_setup" \
    A2W_INSTALL_SETUP="$install_setup" \
    A2W_PC_IP="192.168.123.77" \
    A2W_NETWORK_INTERFACE="test0" \
    A2W_DDS_PEER="192.168.123.164" \
    A2W_RUNTIME_DIR="$runtime_dir" \
    ROS2_BIN="$fake_topic_ros2" \
        "$repository_root/scripts/check_lidar_topics.sh"
)

grep -F 'topic_ros2 PWD=/tmp ARGS=topic info /lidar_points --verbose ' \
    "$tool_capture" >/dev/null || fail "diagnostic did not inspect /lidar_points"
grep -F 'topic_ros2 PWD=/tmp ARGS=topic info /lidar_imu --verbose ' \
    "$tool_capture" >/dev/null || fail "diagnostic did not inspect /lidar_imu"

printf 'PASS: portable launcher behavior\n'

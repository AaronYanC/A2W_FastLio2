#!/usr/bin/env bash
set -euo pipefail

repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
# shellcheck source=scripts/lib/a2w_common.sh
source "$repository_root/scripts/lib/a2w_common.sh"

ros_distro=${ROS_DISTRO:-humble}
ros_setup=${A2W_ROS_SETUP:-"/opt/ros/$ros_distro/setup.bash"}
git_bin=${GIT_BIN:-git}
colcon_bin=${COLCON_BIN:-colcon}
python_executable=${A2W_PYTHON_EXECUTABLE:-/usr/bin/python3}

a2w_require_file "$ros_setup" "ROS setup"
a2w_require_file "$python_executable" "ROS-compatible Python interpreter"
command -v "$git_bin" >/dev/null 2>&1 || a2w_fail "git command not found: $git_bin"
command -v "$colcon_bin" >/dev/null 2>&1 || a2w_fail "colcon command not found: $colcon_bin"

set +u
# shellcheck disable=SC1090
source "$ros_setup"
set -u

"$git_bin" -C "$repository_root" submodule update --init --recursive
cd "$repository_root"
exec "$colcon_bin" build --symlink-install \
    --cmake-args \
    "-DPython3_EXECUTABLE=$python_executable" \
    "-DPYTHON_EXECUTABLE=$python_executable" \
    "$@"

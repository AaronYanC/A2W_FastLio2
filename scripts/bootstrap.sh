#!/usr/bin/env bash
set -euo pipefail

repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
# shellcheck source=scripts/lib/a2w_common.sh
source "$repository_root/scripts/lib/a2w_common.sh"

ros_distro=${ROS_DISTRO:-humble}
ros_setup=${A2W_ROS_SETUP:-"/opt/ros/$ros_distro/setup.bash"}
git_bin=${GIT_BIN:-git}
rosdep_bin=${ROSDEP_BIN:-rosdep}

a2w_require_file "$ros_setup" "ROS setup"
command -v "$git_bin" >/dev/null 2>&1 || a2w_fail "git command not found: $git_bin"
command -v "$rosdep_bin" >/dev/null 2>&1 || a2w_fail "rosdep command not found: $rosdep_bin"

set +u
# shellcheck disable=SC1090
source "$ros_setup"
set -u

"$git_bin" -C "$repository_root" submodule update --init --recursive
"$rosdep_bin" install --from-paths "$repository_root/src" --ignore-src -r -y

printf 'A2W Fast-LIO2 dependencies are ready.\n'

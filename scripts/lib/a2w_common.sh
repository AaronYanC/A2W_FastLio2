#!/usr/bin/env bash

a2w_repository_root() {
    cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P
}

a2w_fail() {
    printf 'A2W Fast-LIO2: %s\n' "$1" >&2
    return 1
}

a2w_require_file() {
    local path=$1
    local description=$2
    [[ -f "$path" ]] || a2w_fail "$description not found: $path"
}

a2w_source_ros_environment() {
    local repository_root=$1
    local ros_distro=${ROS_DISTRO:-humble}
    local ros_setup=${A2W_ROS_SETUP:-"/opt/ros/$ros_distro/setup.bash"}
    local install_setup=${A2W_INSTALL_SETUP:-"$repository_root/install/setup.bash"}

    a2w_require_file "$ros_setup" "ROS setup" || return 1
    a2w_require_file "$install_setup" "workspace setup; run scripts/build.sh first" || return 1

    set +u
    # shellcheck disable=SC1090
    source "$ros_setup"
    # shellcheck disable=SC1090
    source "$install_setup"
    set -u
}

a2w_resolve_network() {
    local peer=$1
    local ip_bin=${A2W_IP_BIN:-ip}
    local python_bin=${PYTHON_BIN:-python3}
    local repository_root
    local address_output
    local route_output
    local selection

    if [[ -n ${A2W_PC_IP:-} && -n ${A2W_NETWORK_INTERFACE:-} ]]; then
        export A2W_PC_IP A2W_NETWORK_INTERFACE
        return 0
    fi

    command -v "$ip_bin" >/dev/null 2>&1 || \
        a2w_fail "ip command not found; set A2W_PC_IP and A2W_NETWORK_INTERFACE" || return 1

    if [[ -n ${A2W_NETWORK_INTERFACE:-} ]]; then
        route_output=$(
            "$ip_bin" -o -4 addr show dev "$A2W_NETWORK_INTERFACE" scope global
        ) || a2w_fail "cannot read IPv4 for interface $A2W_NETWORK_INTERFACE" || return 1
        A2W_PC_IP=$(awk 'NR == 1 { split($4, address, "/"); print address[1] }' <<<"$route_output")
    else
        repository_root=$(a2w_repository_root)
        address_output=$("$ip_bin" -o -4 addr show scope global) || \
            a2w_fail "cannot list PC IPv4 interfaces" || return 1
        if selection=$(printf '%s\n' "$address_output" | \
            "$python_bin" "$repository_root/scripts/select_network_interface.py" --peer "$peer"); then
            read -r A2W_NETWORK_INTERFACE A2W_PC_IP <<<"$selection"
        else
            route_output=$("$ip_bin" -4 route get "$peer") || \
                a2w_fail "no route to DDS peer $peer" || return 1
            A2W_NETWORK_INTERFACE=$(awk '{ for (i = 1; i <= NF; i++) if ($i == "dev") { print $(i + 1); exit } }' <<<"$route_output")
            if [[ -z ${A2W_PC_IP:-} ]]; then
                A2W_PC_IP=$(awk '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }' <<<"$route_output")
            fi
        fi
    fi

    [[ -n ${A2W_NETWORK_INTERFACE:-} ]] || \
        a2w_fail "route detection returned no interface; set A2W_NETWORK_INTERFACE" || return 1
    [[ -n ${A2W_PC_IP:-} ]] || \
        a2w_fail "route detection returned no source IPv4; set A2W_PC_IP" || return 1
    export A2W_PC_IP A2W_NETWORK_INTERFACE
}

a2w_prepare_cyclonedds() {
    local repository_root=$1
    local runtime_dir=${A2W_RUNTIME_DIR:-"$repository_root/log/fast_lio_runtime"}
    local peer=${A2W_DDS_PEER:-192.168.123.164}
    local python_bin=${PYTHON_BIN:-python3}
    local generator="$repository_root/scripts/generate_cyclonedds_config.py"
    local template="$repository_root/config/cyclonedds_unitree_a2.xml.in"
    local output="$runtime_dir/cyclonedds.xml"

    a2w_resolve_network "$peer" || return 1
    "$python_bin" "$generator" \
        --template "$template" \
        --output "$output" \
        --pc-address "$A2W_PC_IP" \
        --peer-address "$peer"

    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="file://$output"
    export ROS_LOG_DIR="$repository_root/log/fast_lio_runtime"
    mkdir -p "$ROS_LOG_DIR"

    printf 'A2W Fast-LIO2 DDS: interface=%s pc=%s peer=%s\n' \
        "$A2W_NETWORK_INTERFACE" "$A2W_PC_IP" "$peer"
}

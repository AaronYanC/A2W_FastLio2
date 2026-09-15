#!/usr/bin/env bash
set -uo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "$script_dir/.." && pwd)"
# shellcheck source=lib/full_validation_checks.sh
source "$script_dir/lib/full_validation_checks.sh"

mode=all
duration=10
output_dir=""
map_bundle=""
dry_run=false
min_lidar_hz=5.0
min_imu_hz=100.0
min_odom_hz=10.0
min_keyframe_hz=0.05
max_sensor_skew_ms=100.0

usage() {
  echo "usage: $0 [--mode mapping|localization|all] [--duration SEC] [--output-dir DIR]"
  echo "          [--map-bundle DIR] [--dry-run] [--min-lidar-hz HZ] [--min-imu-hz HZ]"
  echo "          [--min-odom-hz HZ] [--min-keyframe-hz HZ]"
  echo "          [--max-sensor-skew-ms MS]"
}

while (($#)); do
  case "$1" in
    --mode) mode="${2:-}"; shift 2 ;;
    --duration) duration="${2:-}"; shift 2 ;;
    --output-dir) output_dir="${2:-}"; shift 2 ;;
    --map-bundle) map_bundle="${2:-}"; shift 2 ;;
    --min-lidar-hz) min_lidar_hz="${2:-}"; shift 2 ;;
    --min-imu-hz) min_imu_hz="${2:-}"; shift 2 ;;
    --min-odom-hz) min_odom_hz="${2:-}"; shift 2 ;;
    --min-keyframe-hz) min_keyframe_hz="${2:-}"; shift 2 ;;
    --max-sensor-skew-ms) max_sensor_skew_ms="${2:-}"; shift 2 ;;
    --dry-run) dry_run=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ "$mode" != mapping && "$mode" != localization && "$mode" != all ]]; then
  echo "--mode must be mapping, localization, or all" >&2
  exit 2
fi
if ! [[ "$duration" =~ ^[1-9][0-9]*$ ]]; then
  echo "--duration must be a positive integer" >&2
  exit 2
fi
if [[ -z "$output_dir" ]]; then
  output_dir="$repository_root/log/jt128_validation/$(date -u +%Y%m%dT%H%M%SZ)"
elif [[ "$output_dir" != /* ]]; then
  output_dir="$repository_root/$output_dir"
fi
mkdir -p "$output_dir/raw"

declare -A check_status check_detail
record() {
  local id="$1" status="$2" detail="$3"
  check_status["$id"]="$status"
  check_detail["$id"]="$detail"
  printf '%s\n' "$detail" > "$output_dir/raw/$id.txt"
}

capture() {
  local id="$1"; shift
  local raw="$output_dir/raw/$id.txt"
  if "$@" > "$raw" 2>&1; then
    check_status["$id"]=pass
    check_detail["$id"]="evidence captured"
  else
    check_status["$id"]=fail
    check_detail["$id"]="command failed; inspect raw evidence"
  fi
}

topic_present() {
  grep -Eq "^$1([[:space:]]|$)" "$output_dir/raw/ros2_topic_list.txt"
}

check_topic() {
  local id="$1" topic="$2"
  if topic_present "$topic"; then
    record "$id" pass "required topic is visible: $topic"
  else
    record "$id" fail "required topic is absent: $topic"
  fi
}

check_rate() {
  local id="$1" topic="$2" threshold="$3"
  local raw="$output_dir/raw/$id.txt"
  timeout "${duration}s" ros2 topic hz --window 100 "$topic" > "$raw" 2>&1
  local measured
  measured="$(awk '/average rate:/ {value=$3} END {print value}' "$raw")"
  if [[ -n "$measured" ]] && awk -v value="$measured" -v minimum="$threshold" \
    'BEGIN {exit !(value + 0 >= minimum + 0)}'; then
    check_status["$id"]=pass
    check_detail["$id"]="measured ${measured} Hz, minimum ${threshold} Hz"
  else
    check_status["$id"]=fail
    check_detail["$id"]="frequency missing or below ${threshold} Hz"
  fi
}

check_qos() {
  local raw="$output_dir/raw/endpoint_qos.txt"
  {
    ros2 topic info -v /lidar_points
    ros2 topic info -v /lidar_imu
  } > "$raw" 2>&1
  if [[ "$(grep -Ec 'Publisher count: [1-9][0-9]*' "$raw")" -ge 2 ]] &&
    grep -Eq 'Reliability: (RELIABLE|BEST_EFFORT)' "$raw" &&
    grep -Eq 'Durability: (VOLATILE|TRANSIENT_LOCAL)' "$raw"; then
    check_status[endpoint_qos]=pass
    check_detail[endpoint_qos]="publisher QoS evidence captured for both JT128 Topics"
  else
    check_status[endpoint_qos]=fail
    check_detail[endpoint_qos]="missing publishers or incomplete QoS evidence"
  fi
}

check_process_usage() {
  local id="$1" field="$2"
  local raw="$output_dir/raw/$id.txt"
  ps -eo "pid,$field,comm,args" > "$raw" 2>&1
  if grep -Eiq '(fastlio|mapping_backend|localization_node)' "$raw"; then
    check_status["$id"]=pass
    check_detail["$id"]="SLAM process resource evidence captured"
  else
    check_status["$id"]=fail
    check_detail["$id"]="no SLAM process found in resource evidence"
  fi
}

check_frames() {
  local raw="$output_dir/raw/frame_contract.txt"
  local odom_raw="$output_dir/raw/frame_contract_odom.txt"
  local cloud_raw="$output_dir/raw/frame_contract_cloud.txt"
  local lidar_raw="$output_dir/raw/frame_contract_lidar.txt"
  local imu_raw="$output_dir/raw/frame_contract_imu.txt"
  ros2 topic echo --once /Odometry > "$odom_raw" 2>&1
  ros2 topic echo --once /cloud_registered_body > "$cloud_raw" 2>&1
  ros2 topic echo --once /lidar_points > "$lidar_raw" 2>&1
  ros2 topic echo --once /lidar_imu > "$imu_raw" 2>&1
  {
    echo "Odometry:"; sed -n '1,120p' "$odom_raw"
    echo "Body cloud:"; sed -n '1,80p' "$cloud_raw"
    echo "JT128 points:"; sed -n '1,80p' "$lidar_raw"
    echo "JT128 IMU:"; sed -n '1,80p' "$imu_raw"
  } > "$raw"
  if grep -Eq 'frame_id: (camera_init|"camera_init")' "$odom_raw" &&
    grep -Eq 'child_frame_id: (body|"body")' "$odom_raw" &&
    grep -Eq 'frame_id: (body|"body")' "$cloud_raw" &&
    grep -Eq 'frame_id: .+' "$lidar_raw" && grep -Eq 'frame_id: .+' "$imu_raw"; then
    check_status[frame_contract]=pass
    check_detail[frame_contract]="camera_init to body frame contract observed"
  else
    check_status[frame_contract]=fail
    check_detail[frame_contract]="expected camera_init/body frames were not observed"
  fi
}

check_timestamps() {
  local raw="$output_dir/raw/timestamp_contract.txt"
  local odom_raw="$output_dir/raw/timestamps_odom.txt"
  local cloud_raw="$output_dir/raw/timestamps_cloud.txt"
  local lidar_raw="$output_dir/raw/timestamps_lidar.txt"
  local imu_raw="$output_dir/raw/timestamps_imu.txt"
  local odom_stamps="$output_dir/raw/timestamps_odom_ns.txt"
  local cloud_stamps="$output_dir/raw/timestamps_cloud_ns.txt"
  local lidar_stamps="$output_dir/raw/timestamps_lidar_ns.txt"
  local imu_stamps="$output_dir/raw/timestamps_imu_ns.txt"
  timeout "${duration}s" ros2 topic echo /Odometry --field header.stamp > "$odom_raw" 2>&1 &
  local odom_pid=$!
  timeout "${duration}s" ros2 topic echo /cloud_registered_body --field header.stamp > "$cloud_raw" 2>&1 &
  local cloud_pid=$!
  timeout "${duration}s" ros2 topic echo /lidar_points --field header.stamp > "$lidar_raw" 2>&1 &
  local lidar_pid=$!
  timeout "${duration}s" ros2 topic echo /lidar_imu --field header.stamp > "$imu_raw" 2>&1 &
  local imu_pid=$!
  wait "$odom_pid" || true; wait "$cloud_pid" || true
  wait "$lidar_pid" || true; wait "$imu_pid" || true
  {
    echo "Odometry:"; sed -n '1,120p' "$odom_raw"
    echo "Body cloud:"; sed -n '1,120p' "$cloud_raw"
    echo "JT128 points:"; sed -n '1,120p' "$lidar_raw"
    echo "JT128 IMU:"; sed -n '1,120p' "$imu_raw"
  } > "$raw"
  local monotonic_program='
    /sec:/ {sec=$2}
    /nanosec:/ {
      value=sec*1000000000+$2
      if (count > 0 && value <= previous) bad=1
      previous=value; count++
    }
    END {exit !(count >= 2 && bad == 0)}'
  local extract_program='/sec:/ {sec=$2} /nanosec:/ {printf "%.0f\n", sec*1000000000+$2}'
  awk "$extract_program" "$odom_raw" > "$odom_stamps"
  awk "$extract_program" "$cloud_raw" > "$cloud_stamps"
  awk "$extract_program" "$lidar_raw" > "$lidar_stamps"
  awk "$extract_program" "$imu_raw" > "$imu_stamps"
  local synchronized_pairs
  synchronized_pairs="$(comm -12 <(sort -n "$odom_stamps") <(sort -n "$cloud_stamps") | wc -l)"
  local sensor_skew_ok=false
  if awk -v maximum_ms="$max_sensor_skew_ms" '
    NR==FNR {lidar[count++]=$1; next}
    {
      for (i=0; i<count; ++i) {
        difference=$1-lidar[i]; if (difference<0) difference=-difference
        if (!found || difference<minimum) {minimum=difference; found=1}
      }
    }
    END {exit !(found && minimum <= maximum_ms*1000000)}' "$lidar_stamps" "$imu_stamps"; then
    sensor_skew_ok=true
  fi
  if awk "$monotonic_program" "$odom_raw" && awk "$monotonic_program" "$cloud_raw" &&
    awk "$monotonic_program" "$lidar_raw" && awk "$monotonic_program" "$imu_raw" &&
    [[ "$synchronized_pairs" -ge 1 ]] && $sensor_skew_ok; then
    check_status[timestamp_contract]=pass
    check_detail[timestamp_contract]="monotonic timestamps, synchronized frontend pair, and sensor skew within ${max_sensor_skew_ms} ms"
  else
    check_status[timestamp_contract]=fail
    check_detail[timestamp_contract]="insufficient or non-monotonic timestamp evidence"
  fi
}

capture_topic_pattern() {
  local id="$1" topic="$2" pattern="$3" detail="$4"
  local raw="$output_dir/raw/$id.txt"
  timeout "${duration}s" ros2 topic echo "$topic" > "$raw" 2>&1
  if grep -Eq "$pattern" "$raw"; then
    check_status["$id"]=pass
    check_detail["$id"]="$detail"
  else
    check_status["$id"]=fail
    check_detail["$id"]="required runtime evidence pattern was not observed"
  fi
}

check_relocalization_states() {
  local raw="$output_dir/raw/global_relocalization.txt"
  timeout "${duration}s" ros2 topic echo /localization/status > "$raw" 2>&1
  if grep -q 'state_label: LOST' "$raw" &&
    grep -q 'state_label: RELOCALIZING' "$raw" &&
    grep -q 'state_label: LOCALIZED' "$raw"; then
    check_status[global_relocalization]=pass
    check_detail[global_relocalization]="LOST, RELOCALIZING, and LOCALIZED were all observed"
  else
    check_status[global_relocalization]=fail
    check_detail[global_relocalization]="complete global recovery sequence was not observed"
  fi
}

if $dry_run; then
  for id in "${VALIDATION_CHECK_IDS[@]}"; do
    record "$id" not_run "DRY RUN: $(validation_check_description "$id")"
  done
else
  if command -v ros2 >/dev/null 2>&1; then
    ros2 topic list -t > "$output_dir/raw/ros2_topic_list.txt" 2>&1 || true
    check_topic lidar_points_visibility /lidar_points
    check_topic lidar_imu_visibility /lidar_imu
  else
    printf '' > "$output_dir/raw/ros2_topic_list.txt"
    record lidar_points_visibility fail "ros2 command is unavailable"
    record lidar_imu_visibility fail "ros2 command is unavailable"
  fi
  check_rate lidar_points_frequency /lidar_points "$min_lidar_hz"
  check_rate lidar_imu_frequency /lidar_imu "$min_imu_hz"
  check_qos
  check_frames
  check_timestamps
  check_rate fastlio_odom_frequency /Odometry "$min_odom_hz"

  if [[ "$mode" == mapping || "$mode" == all ]]; then
    check_rate keyframe_frequency /mapping/keyframe_odom "$min_keyframe_hz"
    capture_topic_pattern scan_context_candidates /mapping/registration_status \
      'stage: loop' "loop candidate audit observed"
    capture_topic_pattern quatro_registration /mapping/registration_status \
      'stage: loop' "coarse-to-fine loop registration evidence observed"
    capture_topic_pattern nano_gicp_registration /mapping/registration_status \
      'correspondence_count: [1-9]' "fine registration correspondences observed"
    capture_topic_pattern loop_validation /mapping/registration_status \
      'accepted: true' "validated loop acceptance observed"
    capture_topic_pattern gtsam_graph /mapping/registration_status \
      'stage: pose_graph' "GTSAM optimized graph update observed"
    capture optimized_map_preview ros2 topic echo --once /mapping/optimized_map_preview
  else
    for id in keyframe_frequency scan_context_candidates quatro_registration \
      nano_gicp_registration loop_validation gtsam_graph optimized_map_preview; do
      record "$id" not_applicable "not applicable to mode $mode"
    done
  fi

  capture_topic_pattern map_to_odom /tf \
    'child_frame_id: (camera_init|"camera_init")' "map to camera_init TF observed"
  if [[ -n "$map_bundle" ]]; then
    if [[ "$map_bundle" != /* ]]; then map_bundle="$repository_root/$map_bundle"; fi
    capture map_bundle ros2 run a2w_fastlio_map map_bundle_inspect "$map_bundle"
  else
    record map_bundle fail "--map-bundle is required for full integrity validation"
  fi
  if [[ "$mode" == localization || "$mode" == all ]]; then
    capture_topic_pattern localization /localization/status \
      'state_label: LOCALIZED' "LOCALIZED state observed"
    check_relocalization_states
  else
    record localization not_applicable "not applicable to mode $mode"
    record global_relocalization not_applicable "not applicable to mode $mode"
  fi
  check_process_usage cpu pcpu
  check_process_usage ram rss
  capture latency timeout "${duration}s" ros2 topic delay /Odometry
fi

overall=passed
for id in "${VALIDATION_CHECK_IDS[@]}"; do
  status="${check_status[$id]:-not_run}"
  if [[ "$status" == fail || "$status" == not_run ]]; then overall=pending; fi
done

summary="$output_dir/validation_summary.yaml"
report="$output_dir/validation_report.md"
{
  echo "schema_version: 1"
  echo "mode: $mode"
  echo "duration_seconds: $duration"
  echo "dry_run: $dry_run"
  echo "overall_status: $overall"
  echo "hardware_validation_status: $([[ "$overall" == passed ]] && echo validated || echo pending)"
  echo "checks:"
  for id in "${VALIDATION_CHECK_IDS[@]}"; do
    echo "  $id: ${check_status[$id]:-not_run}"
  done
} > "$summary"
{
  echo "# JT128 Full Validation Report"
  echo
  echo "- Mode: $mode"
  echo "- Duration: $duration seconds"
  echo "- Overall: $overall"
  echo
  echo "| Check | Status | Detail |"
  echo "| --- | --- | --- |"
  for id in "${VALIDATION_CHECK_IDS[@]}"; do
    echo "| $id | ${check_status[$id]:-not_run} | ${check_detail[$id]:-not run} |"
  done
  echo
  if [[ "$overall" == passed ]]; then
    echo "JT128 hardware validation passed."
  else
    echo "JT128 hardware validation pending."
  fi
} > "$report"

echo "Validation evidence: $output_dir"
if [[ "$overall" == passed ]] || $dry_run; then exit 0; fi
exit 1

#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
runner="$repository_root/scripts/run_jt128_full_validation.sh"
fixture_root="$(mktemp -d)"
trap 'rm -rf -- "$fixture_root"' EXIT

dry_output="$fixture_root/dry"
bash "$runner" --dry-run --mode all --duration 1 --output-dir "$dry_output"
grep -Fq 'overall_status: pending' "$dry_output/validation_summary.yaml"
grep -Fq 'JT128 hardware validation pending' "$dry_output/validation_report.md"
for check in \
  lidar_points_frequency lidar_imu_frequency endpoint_qos frame_contract timestamp_contract \
  fastlio_odom_frequency keyframe_frequency scan_context_candidates quatro_registration \
  nano_gicp_registration loop_validation gtsam_graph map_to_odom optimized_map_preview \
  map_bundle localization global_relocalization cpu ram latency; do
  grep -Fq "$check" "$dry_output/validation_summary.yaml"
done

fake_bin="$fixture_root/bin"
mkdir -p "$fake_bin"
cat > "$fake_bin/ros2" <<'EOF'
#!/usr/bin/env bash
if [[ "$*" == "topic list -t" ]]; then
  echo '/Odometry [nav_msgs/msg/Odometry]'
  exit 0
fi
exit 1
EOF
chmod +x "$fake_bin/ros2"

missing_output="$fixture_root/missing"
set +e
PATH="$fake_bin:$PATH" bash "$runner" --mode localization --duration 1 \
  --output-dir "$missing_output"
missing_status=$?
set -e
if [[ $missing_status -eq 0 ]]; then
  echo "validation unexpectedly passed without JT128 topics" >&2
  exit 1
fi
grep -Fq 'overall_status: pending' "$missing_output/validation_summary.yaml"
grep -Fq 'required topic is absent' "$missing_output/raw/lidar_points_visibility.txt"
grep -Fq 'JT128 hardware validation pending' "$missing_output/validation_report.md"
if grep -Fq 'JT128 hardware validation passed' "$missing_output/validation_report.md"; then
  echo "missing hardware was incorrectly marked passed" >&2
  exit 1
fi

cat > "$fake_bin/ros2" <<'EOF'
#!/usr/bin/env bash
if [[ "$*" == "topic list -t" ]]; then
  printf '%s\n' \
    '/lidar_points [sensor_msgs/msg/PointCloud2]' \
    '/lidar_imu [sensor_msgs/msg/Imu]' \
    '/Odometry [nav_msgs/msg/Odometry]'
elif [[ "$*" == topic\ info* ]]; then
  printf '%s\n' 'Publisher count: 1' 'Reliability: RELIABLE' 'Durability: VOLATILE'
elif [[ "$*" == topic\ hz* ]]; then
  echo 'average rate: 200.0'
elif [[ "$*" == *'--field header.stamp'* ]]; then
  printf '%s\n' 'sec: 10' 'nanosec: 1' '---' 'sec: 10' 'nanosec: 2'
elif [[ "$*" == *'/Odometry'* ]]; then
  printf '%s\n' 'frame_id: camera_init' 'child_frame_id: body'
elif [[ "$*" == *'/cloud_registered_body'* ]]; then
  echo 'frame_id: body'
elif [[ "$*" == *'/lidar_points'* || "$*" == *'/lidar_imu'* ]]; then
  echo 'frame_id: jt128'
elif [[ "$*" == *'/mapping/registration_status'* ]]; then
  printf '%s\n' 'stage: loop' 'accepted: true' 'correspondence_count: 100' 'stage: pose_graph'
elif [[ "$*" == *'/tf'* ]]; then
  printf '%s\n' 'frame_id: map' 'child_frame_id: camera_init'
elif [[ "$*" == *'/localization/status'* ]]; then
  printf '%s\n' 'state_label: LOST' 'state_label: RELOCALIZING' 'state_label: LOCALIZED'
else
  echo 'synthetic evidence'
fi
exit 0
EOF
cat > "$fake_bin/timeout" <<'EOF'
#!/usr/bin/env bash
shift
exec "$@"
EOF
cat > "$fake_bin/ps" <<'EOF'
#!/usr/bin/env bash
echo '123 10.0 204800 localization_node localization_node'
EOF
chmod +x "$fake_bin/ros2" "$fake_bin/timeout" "$fake_bin/ps"

success_output="$fixture_root/success"
PATH="$fake_bin:$PATH" bash "$runner" --mode all --duration 1 \
  --map-bundle "$fixture_root/synthetic_bundle" --output-dir "$success_output"
grep -Fq 'overall_status: passed' "$success_output/validation_summary.yaml"
grep -Fq 'hardware_validation_status: validated' "$success_output/validation_summary.yaml"

echo "PASS: full validation runner dry-run, success fixture, and missing-hardware behavior"

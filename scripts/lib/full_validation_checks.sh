#!/usr/bin/env bash

VALIDATION_CHECK_IDS=(
  lidar_points_visibility lidar_imu_visibility lidar_points_frequency lidar_imu_frequency
  endpoint_qos frame_contract timestamp_contract fastlio_odom_frequency keyframe_frequency
  scan_context_candidates quatro_registration nano_gicp_registration loop_validation
  gtsam_graph map_to_odom optimized_map_preview map_bundle localization
  global_relocalization cpu ram latency
)

validation_check_description() {
  case "$1" in
    lidar_points_visibility) echo "JT128 point-cloud Topic visibility" ;;
    lidar_imu_visibility) echo "JT128 IMU Topic visibility" ;;
    lidar_points_frequency) echo "JT128 point-cloud frequency" ;;
    lidar_imu_frequency) echo "JT128 IMU frequency" ;;
    endpoint_qos) echo "DDS endpoint QoS compatibility" ;;
    frame_contract) echo "camera_init/body frame contract" ;;
    timestamp_contract) echo "timestamp monotonicity and skew" ;;
    fastlio_odom_frequency) echo "FAST-LIO odometry frequency" ;;
    keyframe_frequency) echo "Mapping keyframe frequency" ;;
    scan_context_candidates) echo "Scan Context Top-K candidates" ;;
    quatro_registration) echo "Quatro coarse-registration evidence" ;;
    nano_gicp_registration) echo "Nano-GICP fine-registration evidence" ;;
    loop_validation) echo "validated loop evidence" ;;
    gtsam_graph) echo "GTSAM/iSAM2 optimized graph output" ;;
    map_to_odom) echo "single-owner map to camera_init TF continuity" ;;
    optimized_map_preview) echo "downsampled optimized map preview" ;;
    map_bundle) echo "Map Bundle integrity and global_map.pcd" ;;
    localization) echo "Localization state and correction" ;;
    global_relocalization) echo "LOST to RELOCALIZING to LOCALIZED recovery" ;;
    cpu) echo "process CPU usage" ;;
    ram) echo "process RAM usage" ;;
    latency) echo "ROS Topic transport latency" ;;
  esac
}

# Hesai JT128 FAST-LIO2 PC Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the Hesai `FAST_LIO_Hesai` ROS2 branch buildable and verifiably configured for JT128 on the ThinkBook ROS 2 Humble workspace without running a lidar driver.

**Architecture:** Restore the exact pinned `ikd-Tree` Git submodule, build only the `fast_lio` package, then validate the installed package and official JT128 parameter contract. Perform a bounded startup probe with CycloneDDS and no sensor input; waiting for `/lidar_points` and `/lidar_imu` is expected.

**Tech Stack:** Ubuntu 22.04, ROS 2 Humble, `colcon`, CMake, C++17, PCL 1.12, Eigen 3, `rmw_cyclonedds_cpp`

**Spec:** `docs/superpowers/specs/2026-09-02-hesai-jt128-fast-lio2-pc-design.md`

## Global Constraints

- Work only in `/home/dndx/fast_lio_ros2_ws` and its nested `src/FAST_LIO_Hesai` repository.
- Do not modify or stop services on `192.168.123.162` or `192.168.123.164`.
- Do not modify robot networking, lidar destination IP, lidar ports, or lidar parameters.
- Do not stop Unitree's built-in SLAM service.
- Do not run or build Hesai Driver in this phase.
- Do not reuse `/home/dndx/cyclonedds/cyclonedds.xml`, because it pins the obsolete `172.20.10.11` interface.
- Do not change the identity `mapping.extrinsic_T` or `mapping.extrinsic_R` without measured or vendor-provided A2-W Pro extrinsics.
- Do not commit generated build products or process documents to the parent repository rooted above the workspace.

---

### Task 1: Restore the pinned ikd-Tree dependency

**Files:**
- Populate: `src/FAST_LIO_Hesai/include/ikd-Tree/`
- Inspect: `src/FAST_LIO_Hesai/.gitmodules`
- Inspect: `src/FAST_LIO_Hesai/CMakeLists.txt`

**Interfaces:**
- Consumes: Git submodule entry `include/ikd-Tree` pinned by the `FAST_LIO_Hesai` `ROS2` commit.
- Produces: `src/FAST_LIO_Hesai/include/ikd-Tree/ikd_Tree.cpp` and `ikd_Tree.h` at Git commit `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`.

- [x] **Step 1: Confirm the dependency is missing and the superproject is otherwise clean**

  Run:

  ```bash
  test ! -f src/FAST_LIO_Hesai/include/ikd-Tree/ikd_Tree.cpp
  git -C src/FAST_LIO_Hesai status --short
  git -C src/FAST_LIO_Hesai submodule status
  ```

  Expected: the file check succeeds, the main repository has no tracked modifications, and submodule status begins with `-e2e3f4e...`.

- [x] **Step 2: Initialize the exact recorded submodule revision**

  Run:

  ```bash
  git -C src/FAST_LIO_Hesai submodule update --init --recursive
  ```

  Expected: Git checks out `include/ikd-Tree` without changing the superproject's recorded submodule pointer.

- [x] **Step 3: Verify dependency identity and required sources**

  Run:

  ```bash
  test -f src/FAST_LIO_Hesai/include/ikd-Tree/ikd_Tree.cpp
  test -f src/FAST_LIO_Hesai/include/ikd-Tree/ikd_Tree.h
  git -C src/FAST_LIO_Hesai/include/ikd-Tree rev-parse HEAD
  git -C src/FAST_LIO_Hesai submodule status
  git -C src/FAST_LIO_Hesai status --short
  ```

  Expected: submodule HEAD is `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`, submodule status has a leading space rather than `-` or `+`, and the superproject is clean.

### Task 2: Build only the fast_lio package

**Files:**
- Generate: `build/fast_lio/`
- Generate: `install/fast_lio/`
- Generate: `log/build_*/`

**Interfaces:**
- Consumes: ROS 2 Humble environment and the restored `ikd-Tree` source from Task 1.
- Produces: installed executable `install/fast_lio/lib/fast_lio/fastlio_mapping`, launch files, configuration files, and diagnostic tools.

- [x] **Step 1: Confirm ROS and native dependencies are discoverable**

  Run:

  ```bash
  source /opt/ros/humble/setup.bash
  ros2 pkg prefix rclcpp
  ros2 pkg prefix pcl_ros
  ros2 pkg prefix pcl_conversions
  ros2 pkg prefix rmw_cyclonedds_cpp
  pkg-config --modversion eigen3
  dpkg-query -W -f='${Status} ${Version}\n' libpcl-dev
  test -f /usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake
  ```

  Expected: ROS packages resolve under `/opt/ros/humble`; Eigen prints its
  version; `libpcl-dev` reports `install ok installed`; and PCL's CMake package
  file exists.

- [x] **Step 2: Build the selected package**

  Run from `/home/dndx/fast_lio_ros2_ws`:

  ```bash
  source /opt/ros/humble/setup.bash
  colcon build --packages-select fast_lio --symlink-install
  ```

  Expected: `Summary: 1 package finished` with exit status zero. CMake's non-fatal `PCL_ROOT` compatibility warning is acceptable.

- [x] **Step 3: Verify installed ROS package and executable**

  Run:

  ```bash
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 pkg prefix fast_lio
  ros2 pkg executables fast_lio
  test -x install/fast_lio/lib/fast_lio/fastlio_mapping
  ```

  Expected: package prefix is `/home/dndx/fast_lio_ros2_ws/install/fast_lio`, and `fast_lio fastlio_mapping` is listed.

### Task 3: Validate JT128 configuration and bounded startup

**Files:**
- Inspect: `src/FAST_LIO_Hesai/config/jt128.yaml`
- Inspect installed copy: `install/fast_lio/share/fast_lio/config/jt128.yaml`
- Inspect: `src/FAST_LIO_Hesai/launch/mapping_jt128.launch.py`

**Interfaces:**
- Consumes: `/lidar_points` as `sensor_msgs/msg/PointCloud2` and `/lidar_imu` as `sensor_msgs/msg/Imu` when the future `.164` DDS path is available.
- Produces: a verified, launchable FAST-LIO2 node configured with ROS2 lidar enum `2`, 128 scan lines, and automatic IMU gyroscope unit detection.

- [x] **Step 1: Run the repository's static JT128 configuration checker**

  Run:

  ```bash
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  /usr/bin/python3 src/FAST_LIO_Hesai/tools/check_config.py \
    --config src/FAST_LIO_Hesai/config/jt128.yaml --model jt128 --ros 2
  ```

  Expected: all checks pass for `/lidar_points`, `/lidar_imu`, `lidar_type=2`, `scan_line=128`, and timestamp unit seconds.

- [x] **Step 2: Confirm the installed configuration matches the checked source**

  Run:

  ```bash
  cmp src/FAST_LIO_Hesai/config/jt128.yaml \
    install/fast_lio/share/fast_lio/config/jt128.yaml
  ```

  Expected: `cmp` exits zero with no output.

- [x] **Step 3: Start FAST-LIO2 briefly with CycloneDDS and no RViz**

  Run:

  ```bash
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_LOG_DIR="$PWD/log/fast_lio_runtime"
  unset CYCLONEDDS_URI
  timeout --signal=INT 8 ros2 launch fast_lio mapping_jt128.launch.py rviz:=false
  ```

  Expected: the process starts `fastlio_mapping`, prints a configuration summary containing `lid_topic=/lidar_points`, `imu_topic=/lidar_imu`, `lidar_type=2`, and `scan_line=128`, then exits after the controlled timeout. The timeout exit status is acceptable only when the node started without an error or crash.

- [x] **Step 4: Verify only the bounded PC test process was stopped**

  Run:

  ```bash
  pgrep -af 'fastlio_mapping|mapping_jt128.launch.py' || true
  ```

  Expected: no remaining FAST-LIO2 test process. Do not inspect, signal, or stop robot-side processes.

### Task 4: Record the readiness boundary

**Files:**
- Update checkboxes: `docs/superpowers/plans/2026-09-02-hesai-jt128-fast-lio2-pc.md`

**Interfaces:**
- Consumes: verification evidence from Tasks 1–3.
- Produces: a handoff that distinguishes successful PC software readiness from the future `.164` Hesai Driver and DDS data-path work.

- [x] **Step 1: Re-run the compact verification set**

  Run:

  ```bash
  git -C src/FAST_LIO_Hesai submodule status
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 pkg prefix fast_lio
  ros2 pkg executables fast_lio
  /usr/bin/python3 src/FAST_LIO_Hesai/tools/check_config.py \
    --config src/FAST_LIO_Hesai/config/jt128.yaml --model jt128 --ros 2
  ```

  Expected: pinned submodule is initialized, the package and executable resolve, and the JT128 configuration checker passes.

- [x] **Step 2: Report the exact boundary**

  Report PC FAST-LIO2 as ready only if every compact check succeeds. State separately that live mapping remains unavailable until `.164` publishes `/lidar_points` and `/lidar_imu` over a DDS configuration reachable from `192.168.123.100`.

No Git commit is created for these tasks: the only source-tree change is initializing a submodule at the revision already recorded by the official repository, while build artifacts and process documents must not be committed to the parent home-directory repository.

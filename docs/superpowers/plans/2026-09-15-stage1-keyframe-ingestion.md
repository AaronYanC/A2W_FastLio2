# Stage 1 Keyframe Ingestion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a ROS2 Mapping ingress node that synchronizes the existing Hesai FAST-LIO odometry and body-frame registered cloud, then emits stable, threshold-controlled keyframes without changing the FAST-LIO frontend.

**Architecture:** `a2w_fastlio_common` owns ROS-independent pose, frontend-frame, point-cloud, and keyframe types. `a2w_fastlio_mapping` owns a ROS-independent `KeyframeManager` plus a thin ROS2 node that exact-time synchronizes `/Odometry` with `/cloud_registered_body`; accepted keyframes are published for Stage 1 inspection. Mapping, place recognition, registration, GTSAM, map saving, and TF correction are deliberately absent from this stage.

**Tech Stack:** Ubuntu 22.04, ROS2 Humble, C++17, ament_cmake, rclcpp, message_filters, nav_msgs, sensor_msgs, PCL 1.12, Eigen 3.4, GoogleTest/ament_cmake_gtest.

**Spec:** `integration_report.md`

## Global Constraints

- Preserve `src/FAST_LIO_Hesai` at commit `16e97cdcc4260b9ba6241518a19ad8384224ba48`; do not edit its source, JT128 parsing, IMU parsing, ESIKF, ikd-tree, or extrinsic configuration.
- Do not modify robot hosts, robot networking, Hesai Driver settings, or Unitree services.
- Use the existing `/Odometry` and `/cloud_registered_body` outputs; both are stamped with the same FAST-LIO `lidar_end_time` and identify `camera_init`/`body` in current code.
- Keep algorithm classes free of ROS headers and ROS node ownership.
- Use C++17 and system Eigen/PCL; do not upgrade or replace existing system libraries.
- Stage 1 does not import Scan Context, Quatro, Nano-GICP, TEASER++, or GTSAM.
- Map PCD files are runtime artifacts and remain excluded from Git.
- Every behavior change follows red-green-refactor: write a real failing test, observe the expected failure, then add the smallest implementation.
- Commit only Stage 1 files on `feature/slam-localization-backend`; keep commits small and stage-specific.

---

## Proposed File Structure

```text
src/a2w_fastlio_common/
├── CMakeLists.txt
├── package.xml
├── include/a2w_fastlio_common/
│   ├── point_types.hpp          # PointT and Cloud aliases
│   └── types.hpp                # Pose3d, FrontendFrame, KeyFrame
└── test/
    └── test_types.cpp

src/a2w_fastlio_mapping/
├── CMakeLists.txt
├── package.xml
├── include/a2w_fastlio_mapping/
│   ├── frontend_message_adapter.hpp
│   └── keyframe_manager.hpp
├── src/
│   ├── frontend_message_adapter.cpp
│   ├── keyframe_manager.cpp
│   └── mapping_ingress_node.cpp
├── config/mapping.yaml
└── test/
    ├── test_frontend_message_adapter.cpp
    ├── test_keyframe_manager.cpp
    └── test_mapping_ingress_launch.py

src/a2w_fastlio2_bringup/launch/
└── mapping_stage1.launch.py
```

`a2w_fastlio_common` exports only C++ data types and Eigen/PCL dependencies.
`a2w_fastlio_mapping` links that library and contains all ROS conversion/synchronization code.

---

### Task 1: Shared keyframe data contract

**Files:**
- Create: `src/a2w_fastlio_common/CMakeLists.txt`
- Create: `src/a2w_fastlio_common/package.xml`
- Create: `src/a2w_fastlio_common/include/a2w_fastlio_common/point_types.hpp`
- Create: `src/a2w_fastlio_common/include/a2w_fastlio_common/types.hpp`
- Test: `src/a2w_fastlio_common/test/test_types.cpp`

**Interfaces:**
- Produces: `a2w_fastlio_common::PointT`, `Cloud`, `CloudPtr`, `CloudConstPtr`.
- Produces: `Pose3d { Eigen::Quaterniond rotation; Eigen::Vector3d translation; }`.
- Produces: `FrontendFrame { int64_t stamp_ns; Pose3d odom_pose; CloudConstPtr body_cloud; }`.
- Produces: `KeyFrame { uint64_t id; int64_t stamp_ns; Pose3d odom_pose; Pose3d optimized_pose; CloudPtr body_cloud; std::vector<float> scan_context_descriptor; }`.

- [x] **Step 1: Add the package scaffold and failing data-contract test**

Create an ament package that builds this test before either header exists:

```cpp
#include <gtest/gtest.h>
#include "a2w_fastlio_common/types.hpp"

TEST(KeyFrameTypes, DefaultsPoseToIdentityAndDescriptorToEmpty) {
  a2w_fastlio_common::KeyFrame keyframe;
  EXPECT_EQ(keyframe.id, 0U);
  EXPECT_EQ(keyframe.stamp_ns, 0);
  EXPECT_TRUE(keyframe.odom_pose.translation.isZero());
  EXPECT_TRUE(keyframe.odom_pose.rotation.isApprox(Eigen::Quaterniond::Identity()));
  EXPECT_TRUE(keyframe.optimized_pose.translation.isZero());
  EXPECT_TRUE(keyframe.scan_context_descriptor.empty());
}
```

- [x] **Step 2: Run the test and verify RED**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select a2w_fastlio_common
```

Expected: compilation fails because `a2w_fastlio_common/types.hpp` is missing.

- [x] **Step 3: Implement the minimal shared types**

Use `pcl::PointXYZI` for backend storage. Initialize both poses to identity, both cloud pointers to valid empty clouds, and the descriptor to an empty vector. Do not add ROS message types or algorithm methods.

- [x] **Step 4: Run the package test and verify GREEN**

Run:

```bash
colcon build --symlink-install --packages-select a2w_fastlio_common
colcon test --packages-select a2w_fastlio_common --event-handlers console_direct+
colcon test-result --verbose
```

Expected: build succeeds and `KeyFrameTypes.DefaultsPoseToIdentityAndDescriptorToEmpty` passes.

- [x] **Step 5: Commit the shared contract**

```bash
git add src/a2w_fastlio_common
git commit -m "feat(common): add frontend frame and keyframe types"
```

---

### Task 2: Keyframe selection policy

**Files:**
- Create: `src/a2w_fastlio_mapping/CMakeLists.txt`
- Create: `src/a2w_fastlio_mapping/package.xml`
- Create: `src/a2w_fastlio_mapping/include/a2w_fastlio_mapping/keyframe_manager.hpp`
- Create: `src/a2w_fastlio_mapping/src/keyframe_manager.cpp`
- Test: `src/a2w_fastlio_mapping/test/test_keyframe_manager.cpp`

**Interfaces:**
- Consumes: `a2w_fastlio_common::FrontendFrame` and `KeyFrame`.
- Produces: `KeyframeConfig { double translation_threshold_m; double rotation_threshold_rad; double max_interval_s; }`.
- Produces: `enum class KeyframeDecisionReason { kAcceptedFirst, kAcceptedTranslation, kAcceptedRotation, kAcceptedMaxInterval, kRejectedBelowThreshold, kRejectedNonMonotonicTimestamp, kRejectedInvalidPose, kRejectedEmptyCloud }`.
- Produces: `KeyframeDecision { KeyframeDecisionReason reason; std::optional<KeyFrame> keyframe; }`.
- Produces: `KeyframeManager::consider(const FrontendFrame&) -> KeyframeDecision`, `size() -> std::size_t`, and `reset()`.

- [ ] **Step 1: Write failing tests for first-frame acceptance and below-threshold rejection**

Use a real one-point PCL cloud. Assert that the first valid frame becomes keyframe id 0, and a later frame below every threshold returns `kRejectedBelowThreshold` without increasing `size()`.

- [ ] **Step 2: Verify RED**

Run:

```bash
colcon build --symlink-install --packages-select a2w_fastlio_common a2w_fastlio_mapping
```

Expected: compilation fails because `KeyframeManager` does not exist.

- [ ] **Step 3: Implement first-frame acceptance and below-threshold rejection**

Validate configuration values as finite and greater than zero. Normalize valid quaternions before storage. Set `optimized_pose = odom_pose` in Stage 1. Assign ids only to accepted frames.

- [ ] **Step 4: Verify GREEN for the first behavior**

Run the mapping package tests; both new assertions must pass.

- [ ] **Step 5: Write failing table-driven tests for all three OR thresholds**

Use hand-derived inputs and expected reasons:

| Change from last keyframe | Expected reason |
| --- | --- |
| translation exactly `1.0 m` | `kAcceptedTranslation` |
| yaw exactly `10 deg` | `kAcceptedRotation` |
| elapsed exactly `2.0 s` | `kAcceptedMaxInterval` |
| `0.99 m`, `9.9 deg`, `1.99 s` | `kRejectedBelowThreshold` |

The test must establish that comparisons are inclusive (`>=`) and that accepted ids are contiguous.

- [ ] **Step 6: Verify RED, then implement the OR policy**

Compute rotation distance as:

```cpp
2.0 * std::acos(std::clamp(std::abs(q_last.dot(q_now)), 0.0, 1.0))
```

Use the last accepted keyframe, not the last received frame, as the threshold reference.

- [ ] **Step 7: Verify GREEN, then test invalid input paths**

Add independent failing tests for an empty cloud, NaN translation, zero/NaN quaternion, and a timestamp not strictly newer than the last accepted keyframe. Each must return its exact rejection reason and must not mutate count/id state.

- [ ] **Step 8: Implement validation and verify the full package**

Run:

```bash
colcon test --packages-select a2w_fastlio_mapping --event-handlers console_direct+
colcon test-result --verbose
```

Expected: all selection, boundary, invalid-input, and state-mutation tests pass.

- [ ] **Step 9: Commit the keyframe policy**

```bash
git add src/a2w_fastlio_mapping
git commit -m "feat(mapping): add threshold-based keyframe manager"
```

---

### Task 3: Snapshot ownership and point-field conversion

**Files:**
- Create: `src/a2w_fastlio_mapping/include/a2w_fastlio_mapping/frontend_message_adapter.hpp`
- Create: `src/a2w_fastlio_mapping/src/frontend_message_adapter.cpp`
- Test: `src/a2w_fastlio_mapping/test/test_frontend_message_adapter.cpp`
- Modify: `src/a2w_fastlio_mapping/CMakeLists.txt`
- Modify: `src/a2w_fastlio_mapping/package.xml`

**Interfaces:**
- Consumes: `nav_msgs::msg::Odometry` and `sensor_msgs::msg::PointCloud2` with equal stamps.
- Produces: `FrontendMessageResult { bool success; std::string error; FrontendFrame frame; }`.
- Produces: `makeFrontendFrame(const nav_msgs::msg::Odometry&, const sensor_msgs::msg::PointCloud2&, std::string_view expected_odom_frame, std::string_view expected_body_frame) -> FrontendMessageResult`.

- [ ] **Step 1: Write a failing conversion test with literal message data**

Construct odometry with stamp `12.345 s`, frame `camera_init`, child `body`, translation
`[1,2,3]`, identity quaternion. Construct a real `pcl::PointCloud<pcl::PointXYZINormal>`
with one point `[4,5,6,intensity=7]`, convert it using `pcl::toROSMsg`, and set the same stamp
and frame `body`. Assert the result stamp is `12345000000 ns`, pose is literal `[1,2,3]`,
and the stored `PointXYZI` retains `[4,5,6,7]` while ignoring normal/curvature fields.

- [ ] **Step 2: Verify RED**

Expected: compilation fails because `frontend_message_adapter.hpp` does not exist.

- [ ] **Step 3: Implement the minimal conversion**

Reject mismatched timestamps, unexpected non-empty frames, invalid pose values, and empty point
clouds. Use `pcl::fromROSMsg` for the actual field conversion. Deep-copy into an owned cloud so
later callback/message destruction cannot alter a keyframe.

- [ ] **Step 4: Verify GREEN and add ownership mutation test**

After conversion, mutate/destroy the source ROS message and assert the returned PCL cloud remains
unchanged. This catches accidental aliasing rather than asserting implementation details.

- [ ] **Step 5: Add failing rejection tests, then implement each minimal branch**

Use separate tests for:

- stamp mismatch of 1 ns;
- odom frame not equal to configured `camera_init`;
- child frame not equal to configured `body`;
- cloud frame not equal to configured `body`;
- empty cloud.

An empty expected frame parameter disables only that specific frame-name check; timestamp and data
validation always remain active.

- [ ] **Step 6: Run both packages and commit**

```bash
colcon build --symlink-install --packages-select a2w_fastlio_common a2w_fastlio_mapping
colcon test --packages-select a2w_fastlio_common a2w_fastlio_mapping --event-handlers console_direct+
colcon test-result --verbose
git add src/a2w_fastlio_mapping
git commit -m "feat(mapping): adapt synchronized FAST-LIO messages"
```

---

### Task 4: ROS2 Mapping ingress node

**Files:**
- Create: `src/a2w_fastlio_mapping/src/mapping_ingress_node.cpp`
- Create: `src/a2w_fastlio_mapping/config/mapping.yaml`
- Test: `src/a2w_fastlio_mapping/test/test_mapping_ingress_launch.py`
- Modify: `src/a2w_fastlio_mapping/CMakeLists.txt`
- Modify: `src/a2w_fastlio_mapping/package.xml`

**Interfaces:**
- Subscribes: configurable `odom_topic` default `/Odometry`.
- Subscribes: configurable `body_cloud_topic` default `/cloud_registered_body`.
- Synchronization: ROS2 `message_filters::TimeSynchronizer` with configurable `sync_queue_size`, default 20.
- Publishes: configurable `/mapping/keyframe_odom` (`nav_msgs/msg/Odometry`).
- Publishes: configurable `/mapping/keyframe_cloud` (`sensor_msgs/msg/PointCloud2`).
- Parameters: `translation_threshold_m=1.0`, `rotation_threshold_deg=10.0`, `max_interval_s=2.0`, `odom_frame=camera_init`, `tracking_frame=body`, input/output topic names, queue size.
- Does not publish TF and does not create a map.

- [ ] **Step 1: Write a failing launch/integration test**

Launch only `mapping_ingress_node`. Publish a matched odom/body-cloud pair using reliable QoS and
literal stamp/frame values. Subscribe to both keyframe outputs and assert one message arrives with
unchanged stamp and frames. Publish a second below-threshold pair and assert no second keyframe is
received during a bounded timeout.

- [ ] **Step 2: Verify RED**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to a2w_fastlio_mapping
```

Expected: build or launch test fails because `mapping_ingress_node` is not installed.

- [ ] **Step 3: Implement the minimal node**

Use two `message_filters::Subscriber` objects and exact-time synchronization because the current
frontend assigns the same `lidar_end_time` to both outputs. Keep subscriber QoS compatible with the
frontend's reliable depth-20 publishers. The synchronized callback calls `makeFrontendFrame`, then
`KeyframeManager::consider`; publish only accepted keyframes. Log rejection counters periodically,
not once per scan.

- [ ] **Step 4: Verify GREEN**

Run the integration test and all package tests. Confirm the node exits cleanly and no TF is
published by this package.

- [ ] **Step 5: Add failing parameter-validation tests**

Launch with zero/negative thresholds and queue size zero. Assert startup fails with a clear
parameter error rather than silently accepting unusable values. Then add the smallest validation
needed for the tests to pass.

- [ ] **Step 6: Commit the ROS2 ingress**

```bash
git add src/a2w_fastlio_mapping
git commit -m "feat(mapping): add ROS2 FAST-LIO keyframe ingress"
```

---

### Task 5: Stage 1 bringup without changing the frontend

**Files:**
- Create: `src/a2w_fastlio2_bringup/launch/mapping_stage1.launch.py`
- Modify: `src/a2w_fastlio2_bringup/CMakeLists.txt`
- Modify: `src/a2w_fastlio2_bringup/package.xml`
- Modify: `tests/test_portable_launchers.sh`
- Modify: `README.md`

**Interfaces:**
- Launches the existing `fast_lio/fastlio_mapping` with the unchanged JT128 YAML.
- Launches `a2w_fastlio_mapping/mapping_ingress_node` with `mapping.yaml`.
- Exposes `rviz`, `save_frontend_pcd`, `frontend_map_file`, and mapping-config arguments.
- Does not launch Localization, publish `map→odom`, or save a Map Bundle.

- [ ] **Step 1: Extend the launcher behavior test first**

Use the existing fake `ros2` harness to invoke the new launch from a directory outside the repo.
Assert arguments use package/config resolution and contain no machine-specific path. Verify the
test fails because the launch file does not yet exist.

- [ ] **Step 2: Add the minimal launch and package dependencies**

Resolve package shares with `ament_index_python`. Keep frontend PCD saving disabled by default.
Do not duplicate JT128 parameters or start another FAST_LIO executable.

- [ ] **Step 3: Verify launcher tests and inspect launch description**

```bash
./tests/test_portable_launchers.sh
source install/setup.bash
ros2 launch a2w_fastlio2_bringup mapping_stage1.launch.py --show-args
```

Expected: portable launcher test passes and all arguments resolve without connecting to a robot.

- [ ] **Step 4: Document only Stage 1 commands and limitations**

Add a README section showing the launch command and inspection topics. Explicitly state:

```text
Stage 1 only creates keyframes.
No loop closure, GTSAM, optimized map, localization, relocalization, or map→odom exists yet.
```

- [ ] **Step 5: Commit bringup and docs**

```bash
git add src/a2w_fastlio2_bringup tests/test_portable_launchers.sh README.md
git commit -m "feat(bringup): launch Stage 1 keyframe ingestion"
```

---

### Task 6: Stage 1 verification and handoff

**Files:**
- Modify only if verification exposes a tested defect in Stage 1 files.

**Interfaces:**
- Produces a buildable offline Stage 1 checkpoint; no runtime claim about the disconnected robot.

- [ ] **Step 1: Confirm forbidden files are unchanged**

```bash
git diff main...HEAD -- src/FAST_LIO_Hesai src/a2w_fastlio2_bringup/config/jt128.yaml maps
```

Expected: no output.

- [ ] **Step 2: Run full build and tests**

```bash
./scripts/build.sh
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
./tests/run_tests.sh
```

Expected: zero build/test failures. Existing portability, DDS generation, path hygiene, and launcher
tests remain green.

- [ ] **Step 3: Inspect repository and dependency boundary**

```bash
git status --short
git submodule status --recursive
git diff --check main...HEAD
```

Expected: clean status; existing FAST_LIO/ikd-tree commits unchanged; no whitespace errors.

- [ ] **Step 4: Record offline limitation and push**

Report that message-level and launch integration tests passed, while real JT128 rate/QoS/frame
validation remains pending until the PC reconnects to the robot. Push the Stage 1 commits to
`origin/feature/slam-localization-backend`; do not merge to `main` automatically.

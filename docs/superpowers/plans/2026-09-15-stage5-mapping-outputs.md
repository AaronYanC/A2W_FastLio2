# Stage 5 Mapping Outputs and Global TF Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish optimized odometry/path, a bounded map preview, and the unique `map -> camera_init` transform.

**Architecture:** Pure transform and map-building libraries are tested independently; a ROS 2 Mapping node adapts results to messages. A DDS ownership announcement prevents two project TF owners from remaining active.

**Tech Stack:** ROS 2 Humble, tf2_ros, nav_msgs, sensor_msgs, visualization_msgs, PCL, launch_testing.

**Spec:** `docs/superpowers/specs/2026-09-15-a2w-slam-localization-backend-design.md`

## Global Constraints

- Publish `/mapping/optimized_map_preview`, never a high-density transient-local full map.
- The complete map goes only to the Stage 6 Map Bundle API.
- Preserve `map -> camera_init -> body`; do not invent `odom/base_link` aliases.
- One project global TF owner per running profile.

---

### Task 1: Map-to-local-odom transform math

**Files:**
- Create: `src/a2w_fastlio_mapping/include/a2w_fastlio_mapping/map_odom_manager.hpp`
- Create: `src/a2w_fastlio_mapping/src/map_odom_manager.cpp`
- Test: `src/a2w_fastlio_mapping/test/test_map_odom_manager.cpp`

**Interfaces:**
- Consumes: synchronized FAST-LIO and optimized body poses.
- Produces: `MapOdomCorrection` with `T_map_camera_init`, source ID/stamp, and age.

- [x] **Step 1: Write RED tests** for `T_map_camera_init = T_map_body_optimized * inverse(T_camera_init_body_fastlio)`, identity, rotation/translation composition, finite validation, and timestamp monotonicity.
- [x] **Step 2: Run RED**; expect missing manager.
- [x] **Step 3: Implement immutable correction snapshots** with the exact update API:

```cpp
MapOdomCorrection update(
  std::uint64_t keyframe_id, std::int64_t stamp_ns,
  const Pose3d & map_body, const Pose3d & camera_init_body);
std::optional<MapOdomCorrection> latest() const;
```
- [x] **Step 4: Run GREEN** with transform tolerances below `1e-9` for analytic cases.

### Task 2: Optimized map builder and bounded preview

**Files:**
- Create: `src/a2w_fastlio_mapping/include/a2w_fastlio_mapping/optimized_map_builder.hpp`
- Create: `src/a2w_fastlio_mapping/src/optimized_map_builder.cpp`
- Test: `src/a2w_fastlio_mapping/test/test_optimized_map_builder.cpp`

**Interfaces:**
- Consumes: `KeyFrameProvider` plus `OptimizedPoseSnapshot`.
- Produces: `buildFull()` for Map Bundle input and `buildPreview()` for bounded DDS output.

- [x] **Step 1: Write RED tests** for optimized-pose application, voxel filtering, deterministic max-point cap, full-map output separate from preview, and no mutation of keyframe clouds.
- [x] **Step 2: Run RED**; expect missing builder.
- [x] **Step 3: Implement `buildFull()` and `buildPreview()`**; preview always applies configured voxel and point limits, while full output is returned in memory for Map Bundle writing and never assigned transient-local DDS QoS.
- [x] **Step 4: Run GREEN** including a large synthetic sequence whose preview stays bounded.

### Task 3: ROS messages and global TF ownership

**Files:**
- Create: `src/a2w_fastlio_msgs/{CMakeLists.txt,package.xml,msg/RegistrationStatus.msg,msg/GlobalTfOwner.msg}`
- Create: `src/a2w_fastlio_mapping/src/mapping_backend_node.cpp`
- Create: `src/a2w_fastlio_mapping/include/a2w_fastlio_mapping/global_tf_ownership.hpp`
- Test: `src/a2w_fastlio_mapping/test/test_mapping_outputs_launch.py`

**Interfaces:**
- Produces: `/mapping/optimized_odom`, `/mapping/optimized_path`,
  `/mapping/optimized_map_preview`, `/mapping/registration_status`, and `map -> camera_init`.
- Uses: `/a2w_fastlio/global_tf_owner` as the parameterized ownership announcement default.

- [x] **Step 1: Write RED launch tests** that feed synthetic keyframes and assert optimized message frames/stamps, `/mapping/optimized_map_preview` QoS and point bound, one `map -> camera_init` TF, and no `/mapping/optimized_map` Topic.
- [x] **Step 2: Add a RED conflict test** launching two owner candidates; assert neither conflict state continues publishing global TF after discovery.
- [x] **Step 3: Implement ownership announcement** on a configurable reliable/transient-local Topic with owner ID, mode, heartbeat, and conflict window. Foreign active ownership disables TF and publishes a fault.

```text
GlobalTfOwner.msg:
builtin_interfaces/Time stamp
string owner_id
string mode
string parent_frame
string child_frame
bool active
```
- [x] **Step 4: Implement ROS output adapters** with parameter-built QoS. Keep graph/registration algorithms outside the node class.
- [x] **Step 5: Run GREEN** under CycloneDDS.

### Task 4: Mapping launch mode and checkpoint

**Files:**
- Create: `src/a2w_fastlio2_bringup/launch/mapping.launch.py`
- Create: `src/a2w_fastlio_mapping/config/mapping_topics.yaml`
- Modify: `README.md`, launcher scripts, package manifests

**Interfaces:**
- Produces: `mapping.launch.py` with exactly one global TF owner and parameterized config paths.

- [x] **Step 1: Write RED static launch tests** asserting exactly one global owner and no Localization backend in Mapping mode.
- [x] **Step 2: Add the launch/config implementation** and mark all runtime facts as hardware pending.
- [x] **Step 3: Run full workspace build/tests, `--show-args`, path hygiene, and protected diffs.**
- [ ] **Step 4: Commit and push**:

```bash
git add src/a2w_fastlio_msgs src/a2w_fastlio_mapping src/a2w_fastlio2_bringup scripts tests README.md
git commit -m "feat(mapping): publish optimized outputs and global TF"
git push origin feature/slam-localization-backend
```

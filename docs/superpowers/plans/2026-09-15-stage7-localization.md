# Stage 7 Localization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement read-only map localization with nearby-keyframe selection, coarse/fine matching, low-rate correction, and high-rate global pose output.

**Architecture:** `a2w_fastlio_localization` consumes Map Bundle snapshots and only common abstract algorithms. Normal localization uses the current corrected estimate to select a temporary local target map.

**Tech Stack:** C++17, ROS 2 Humble, common registration interfaces, Map Bundle V1, tf2, GoogleTest, launch_testing.

**Spec:** `docs/superpowers/specs/2026-09-15-a2w-slam-localization-backend-design.md`

## Global Constraints

- Localization depends on `common + map + msgs` and does not modify any loaded map state.
- No Localization source may include Scan Context, Quatro, Nano-GICP, or TEASER concrete headers.
- Preserve `map -> camera_init -> body` and use the Stage 5 ownership protocol.

---

### Task 1: Local map selection and matching manager

**Files:**
- Create: `src/a2w_fastlio_localization/{CMakeLists.txt,package.xml}`
- Create: `src/a2w_fastlio_localization/include/a2w_fastlio_localization/{local_map_selector.hpp,map_matcher.hpp}`
- Create: corresponding `src/*.cpp`
- Test: `src/a2w_fastlio_localization/test/test_local_map_selector.cpp`
- Test: `src/a2w_fastlio_localization/test/test_map_matcher.cpp`

**Interfaces:**
- Consumes: read-only `MapSnapshot` and only common algorithm interfaces.
- Produces: `SelectionResult select(global_pose)` and `MapMatchResult match(frame,selection)`.

- [ ] **Step 1: Write RED tests** for radius/K-nearest selection, deterministic tie order, boundary selection, no-neighbor failure, coarse-to-fine interface calls, validator rejection, and candidate metrics.
- [ ] **Step 2: Run RED**; expect missing package.
- [ ] **Step 3: Implement selection against immutable optimized poses** and matching through `LocalMapBuilder`, `CoarseRegistration`, `FineRegistration`, and `MatchValidator` interfaces.

```cpp
SelectionResult select(const Pose3d & map_body) const;
MapMatchResult match(
  const FrontendFrame & frame, const SelectionResult & selection) const;
```
- [ ] **Step 4: Run GREEN** with fake algorithms plus one real synthetic registration integration case.

### Task 2: LocalizationManager correction semantics

**Files:**
- Create: `src/a2w_fastlio_localization/include/a2w_fastlio_localization/localization_manager.hpp`
- Create: `src/a2w_fastlio_localization/src/localization_manager.cpp`
- Test: `src/a2w_fastlio_localization/test/test_localization_manager.cpp`

**Interfaces:**
- Consumes: `FrontendFrame` and optional validated `MapMatchResult`.
- Produces: `LocalizationOutput` with global body pose, correction stamp/age, and match evidence.

- [ ] **Step 1: Write RED tests** for initial correction, low-rate matching schedule, high-rate `T_map_body = T_map_camera_init * T_camera_init_body`, failed-match correction age, monotonic timestamps, correction jump rejection, and thread-safe immutable snapshots.
- [ ] **Step 2: Run RED**; expect missing manager.
- [ ] **Step 3: Implement manager state** with injected clock/matcher for deterministic tests. Only validated matches update correction stamp and transform.
- [ ] **Step 4: Run GREEN**, including a sequence where outputs remain continuous while correction age increases after failure.

### Task 3: ROS 2 Localization node and mode launch

**Files:**
- Create: `src/a2w_fastlio_localization/src/localization_node.cpp`
- Create: `src/a2w_fastlio_localization/config/{localization.yaml,map_matching.yaml,localization_topics.yaml}`
- Create: `src/a2w_fastlio2_bringup/launch/localization.launch.py`
- Test: `src/a2w_fastlio_localization/test/test_localization_launch.py`

**Interfaces:**
- Produces: configured Localization Topics and the sole `map -> camera_init` owner in this mode.

- [ ] **Step 1: Write RED launch tests** loading a generated Bundle, publishing exact-stamp odom/cloud pairs, and checking pose/odom/path frames, stamps, configured QoS, global TF, and absence of Mapping backend/owner.
- [ ] **Step 2: Add RED read-only assertions** comparing the bundle manifest before and after the run.
- [ ] **Step 3: Implement node and launch** using Stage 1 adaptation semantics and Stage 5 ownership. Config file paths resolve through package share/project arguments, never developer absolute paths.
- [ ] **Step 4: Run GREEN** under CycloneDDS without a robot.

### Task 4: Stage 7 checkpoint

**Interfaces:**
- Produces: installed `localization.launch.py` and documented offline invocation.

- [ ] **Step 1: Add documentation and profile tests** for `mapping.launch.py` versus `localization.launch.py` mutual exclusion.
- [ ] **Step 2: Run full workspace build/tests, Map Bundle verifier, launch `--show-args`, path hygiene, and protected diffs.**
- [ ] **Step 3: Commit and push**:

```bash
git add src/a2w_fastlio_localization src/a2w_fastlio2_bringup tests scripts README.md
git commit -m "feat(localization): add read-only map matching mode"
git push origin feature/slam-localization-backend
```

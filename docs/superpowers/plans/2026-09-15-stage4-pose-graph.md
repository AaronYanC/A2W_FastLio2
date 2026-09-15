# Stage 4 Pose Graph Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a GTSAM 4.1.1 iSAM2 pose graph with prior, odometry, and validated loop factors.

**Architecture:** A ROS-independent Mapping library owns graph state. Only accepted `LoopConstraint` values enter GTSAM; optimized poses are snapshots keyed by contiguous keyframe IDs.

**Tech Stack:** C++17, GTSAM 4.1.1, Eigen, GoogleTest, ament_cmake.

**Spec:** `docs/superpowers/specs/2026-09-15-a2w-slam-localization-backend-design.md`

## Global Constraints

- Use system GTSAM; do not vendor or rebuild it.
- Do not feed graph corrections into FAST-LIO ESIKF or ikd-tree.
- Reject invalid/non-finite/out-of-order factors before mutating graph state.

---

### Task 1: Pose graph API and transform conversion

**Files:**
- Create: `src/a2w_fastlio_mapping/include/a2w_fastlio_mapping/pose_graph_optimizer.hpp`
- Create: `src/a2w_fastlio_mapping/src/pose_graph_optimizer.cpp`
- Test: `src/a2w_fastlio_mapping/test/test_pose_graph_optimizer.cpp`

**Interfaces:**
- Consumes: contiguous keyframe IDs, odometry `Pose3d`, and accepted `LoopConstraint`.
- Produces: immutable `OptimizedPoseSnapshot` values.

- [x] **Step 1: Write RED tests** for lossless `Pose3d <-> gtsam::Pose3`, first-node prior, adjacent odometry factors, contiguous IDs, and non-finite pose rejection.
- [x] **Step 2: Run RED** with `colcon test --packages-select a2w_fastlio_mapping`; expect missing optimizer.
- [x] **Step 3: Implement the transactional API**:

```cpp
GraphUpdate addKeyFrame(std::uint64_t id, const Pose3d & odom_pose);
GraphUpdate addLoopConstraint(const LoopConstraint & constraint);
GraphUpdate update();
OptimizedPoseSnapshot optimizedPoses() const;
```
- [x] **Step 4: Run GREEN** against system GTSAM 4.1.1.

### Task 2: Synthetic loop closure and robust noise

**Files:**
- Test: `src/a2w_fastlio_mapping/test/test_pose_graph_synthetic_loop.cpp`
- Modify: optimizer header/source

**Interfaces:**
- Extends: `PoseGraphConfig` with prior/odom/loop sigmas, robust kernel, and iSAM2 settings.

- [x] **Step 1: Write a RED square-loop test** whose drifting odometry leaves a measurable endpoint error; after a correct loop factor, assert lower endpoint and total trajectory error. Add a bad loop test whose robust loss limits displacement.
- [x] **Step 2: Run RED**; expect absent loop behavior or unchanged error.
- [x] **Step 3: Add iSAM2 updates**, configurable diagonal prior/odom/loop sigmas, relinearization threshold/skip, and Huber/Cauchy robust loop noise.
- [x] **Step 4: Run GREEN** repeatedly to confirm deterministic snapshots.

### Task 3: Mapping loop coordinator

**Files:**
- Create: `src/a2w_fastlio_mapping/include/a2w_fastlio_mapping/loop_pipeline.hpp`
- Create: `src/a2w_fastlio_mapping/src/loop_pipeline.cpp`
- Test: `src/a2w_fastlio_mapping/test/test_loop_pipeline.cpp`

**Interfaces:**
- Consumes: `KeyFrame`, `DescriptorIndex`, `RegistrationPipeline`, `LoopValidator`, and
  `PoseGraphOptimizer`.
- Produces: `LoopPipelineEvent` audit records and accepted graph constraints.

- [x] **Step 1: Write RED tests** with fake abstract place/coarse/fine algorithms proving Top-K iteration, validator gating, one accepted loop per policy window, and no graph mutation for rejected candidates.
- [x] **Step 2: Run RED**; expect missing coordinator.
- [x] **Step 3: Implement the coordinator** using dependency injection and bounded work items. It emits candidate/registration/loop events but contains no ROS publisher.

```cpp
std::vector<LoopPipelineEvent> process(const KeyFrame & current);
bool enqueue(KeyFrame keyframe);
std::size_t pending() const noexcept;
```
- [x] **Step 4: Run GREEN** and ThreadSanitizer-compatible queue tests where available.

### Task 4: Stage 4 configuration and checkpoint

**Files:**
- Create: `src/a2w_fastlio_mapping/config/pose_graph.yaml`
- Modify: package CMake/package.xml, README, tests

**Interfaces:**
- Produces: installed Mapping graph library and `pose_graph.yaml` consumed by Stage 5.

- [x] **Step 1: Add RED tests** for invalid noise values and missing GTSAM dependency declaration.
- [x] **Step 2: Add every graph parameter and document coordinate conventions.**
- [x] **Step 3: Run full workspace verification and protected-path diff checks.**
- [x] **Step 4: Commit and push**:

```bash
git add src/a2w_fastlio_mapping tests README.md
git commit -m "feat(mapping): add incremental pose graph optimization"
git push origin feature/slam-localization-backend
```

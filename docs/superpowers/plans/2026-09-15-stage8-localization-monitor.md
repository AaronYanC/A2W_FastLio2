# Stage 8 Localization Monitor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an observable, hysteretic Localization state machine covering initialization, degradation, loss, relocalization, and recovery.

**Architecture:** A pure deterministic `LocalizationMonitor` consumes timestamped match evidence and commands. The ROS node publishes its snapshots but does not define transition logic.

**Tech Stack:** C++17, ROS 2 messages, GoogleTest, launch_testing.

**Spec:** `docs/superpowers/specs/2026-09-15-a2w-slam-localization-backend-design.md`

## Global Constraints

- State sequence includes `INITIALIZING`, `LOCALIZED`, `DEGRADED`, `LOST`, and `RELOCALIZING`.
- All counters, durations, quality thresholds, and hysteresis are YAML parameters.
- A failed match never refreshes correction validity.

---

### Task 1: State and evidence types

**Files:**
- Create: `src/a2w_fastlio_localization/include/a2w_fastlio_localization/localization_monitor.hpp`
- Create: `src/a2w_fastlio_localization/src/localization_monitor.cpp`
- Test: `src/a2w_fastlio_localization/test/test_localization_monitor.cpp`

**Interfaces:**
- Consumes: monotonic `MatchEvidence` and explicit relocalization commands.
- Produces: immutable `LocalizationStatusSnapshot`.

- [x] **Step 1: Write a RED transition-table test** covering initialization success, intermittent failures, consecutive degradation, stale correction loss, explicit relocalization start, failed attempts, strong recovery, multi-frame recovery, and reset.
- [x] **Step 2: Add RED tests** for non-monotonic time, invalid configuration, boundary thresholds, hysteresis, and no impossible direct `LOST -> LOCALIZED` transition without accepted relocalization evidence.
- [x] **Step 3: Implement the deterministic transition API**:

```cpp
LocalizationStatusSnapshot update(const MatchEvidence & evidence);
LocalizationStatusSnapshot beginRelocalization(std::int64_t stamp_ns);
LocalizationStatusSnapshot reset(std::int64_t stamp_ns);
```
- [x] **Step 4: Run GREEN** and property-style generated event sequences asserting state invariants.

### Task 2: ROS status message and integration

**Files:**
- Create: `src/a2w_fastlio_msgs/msg/LocalizationStatus.msg`
- Create: `src/a2w_fastlio_localization/config/relocalization.yaml`
- Modify: Localization node/manager
- Test: `src/a2w_fastlio_localization/test/test_localization_status_launch.py`

**Interfaces:**
- Produces: `/localization/status` with state, reason, counters, correction age, match metrics,
  and `hardware_validation_pending=true` until final hardware validation.

- [x] **Step 1: Write RED launch tests** for status enum, reason, failure/success counters, correction age, candidate ID, fitness, overlap, and hardware-validation-pending flag.
- [x] **Step 2: Integrate monitor snapshots** so status timestamps come from processed data/clock and the node cannot publish `LOCALIZED` without accepted evidence.
- [x] **Step 3: Run GREEN** with injected timeout and recovery sequences.

### Task 3: Stage 8 checkpoint

**Interfaces:**
- Produces: documented state table and fully parameterized transition configuration.

- [x] **Step 1: Document every transition and parameter with a state table.**
- [x] **Step 2: Run full workspace build/tests, offline launch tests, path/protected checks.**
- [x] **Step 3: Commit and push**:

```bash
git add src/a2w_fastlio_msgs src/a2w_fastlio_localization tests README.md
git commit -m "feat(localization): add localization health state machine"
git push origin feature/slam-localization-backend
```

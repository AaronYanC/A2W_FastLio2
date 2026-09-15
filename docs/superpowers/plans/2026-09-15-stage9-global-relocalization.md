# Stage 9 Global Relocalization and Validation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Recover from LOST using full-map Scan Context Top-K followed by common coarse/fine registration, validation, ambiguity handling, and multi-frame confirmation.

**Architecture:** `GlobalRelocalizer` is an orchestration layer over abstract common interfaces and read-only Map databases. The final hardware script records all runtime evidence but does not fabricate success when hardware is absent.

**Tech Stack:** C++17, ROS 2 Humble, Map Bundle V1, Scan Context/Quatro/Nano-GICP interfaces, bash validation runner, GoogleTest, launch_testing.

**Spec:** `docs/superpowers/specs/2026-09-15-a2w-slam-localization-backend-design.md`

## Global Constraints

- Never accept Scan Context Top-1 directly.
- Evaluate configured Top-K through candidate local map, coarse registration, fine registration, and validator.
- Mapping/Localization/Relocalization share the same common algorithm objects and result types.
- Report final software state exactly: `Offline implementation and verification complete; JT128 hardware validation pending.`

---

### Task 1: GlobalRelocalizer candidate evaluation

**Files:**
- Create: `src/a2w_fastlio_localization/include/a2w_fastlio_localization/global_relocalizer.hpp`
- Create: `src/a2w_fastlio_localization/src/global_relocalizer.cpp`
- Test: `src/a2w_fastlio_localization/test/test_global_relocalizer.cpp`

**Interfaces:**
- Consumes: full-map `DescriptorIndex`, `KeyFrameProvider`, and abstract registration interfaces.
- Produces: `RelocalizationResult` plus one `CandidateAudit` for every evaluated candidate.

- [ ] **Step 1: Write RED tests** proving all Top-K candidates are evaluated until policy completion; SC rank alone cannot win; invalid candidate maps are skipped with reasons; best/second-best margin rejects ambiguity; strong evidence selects a transform.
- [ ] **Step 2: Run RED**; expect missing relocalizer.
- [ ] **Step 3: Implement candidate evaluation** using only `DescriptorIndex`, `LocalMapBuilder`, `CoarseRegistration`, `FineRegistration`, and `MatchValidator` interfaces. Return a ranked audit record for every candidate.

```cpp
RelocalizationResult evaluate(
  const CloudConstPtr & current_local_map,
  const ScanDescriptor & query_descriptor,
  const RelocalizationConfig & config) const;
```
- [ ] **Step 4: Run GREEN** with fake interfaces and deterministic ordering.

### Task 2: Multi-frame recovery and repeated structures

**Files:**
- Modify: global relocalizer header/source
- Test: `src/a2w_fastlio_localization/test/test_relocalization_sequences.cpp`
- Test data: `src/a2w_fastlio_localization/test/data/repeated_factory_layout.yaml`

**Interfaces:**
- Consumes: per-frame `RelocalizationResult`.
- Produces: `RelocalizationSessionSnapshot` and an optional confirmed correction.

- [ ] **Step 1: Write RED synthetic kidnapped-robot tests** with two geometrically similar candidate areas. Assert the false candidate has a good SC score but fails geometry/ambiguity, while consistent transforms across configured frames restore the true pose.
- [ ] **Step 2: Add RED tests** for timeout, cancellation, changing candidate winner, one strong-result recovery, N-frame recovery, and transform consistency thresholds.
- [ ] **Step 3: Implement a bounded session state** that retains only candidate audit summaries and accepted-transform evidence required by the confirmation policy.
- [ ] **Step 4: Run GREEN** and confirm monitor transitions `LOST -> RELOCALIZING -> LOCALIZED` only after acceptance.

### Task 3: ROS integration and offline end-to-end test

**Files:**
- Modify: Localization node and launch
- Create: `src/a2w_fastlio_localization/test/test_global_relocalization_launch.py`

**Interfaces:**
- Integrates: LOST/RELOCALIZING monitor states, candidate audit publication, and correction update.

- [ ] **Step 1: Write RED launch test** loading a synthetic Bundle, inducing LOST, moving the input sequence to another map region, and asserting candidate audit statuses, no TF update during rejection, then a consistent restored `map -> camera_init`.
- [ ] **Step 2: Implement worker scheduling** with bounded queue, cancellation on shutdown/new session, and parameterized Top-K/time budgets.
- [ ] **Step 3: Run GREEN** under CycloneDDS and verify the Bundle remains byte-identical.

### Task 4: Consolidated JT128 hardware validation runner

**Files:**
- Create: `scripts/run_jt128_full_validation.sh`
- Create: `scripts/lib/full_validation_checks.sh`
- Test: `tests/test_full_validation_runner.sh`
- Modify: `README.md`

**Interfaces:**
- Accepts: `--mode`, `--duration`, `--output-dir`, `--dry-run`, and threshold/config overrides.
- Produces: raw evidence, `validation_summary.yaml`, and `validation_report.md`.

- [ ] **Step 1: Write RED shell tests** with fake `ros2`, `ps`, and `/proc` fixtures. Assert checks cover Topic visibility/rate, endpoint QoS, frames, timestamp monotonicity/skew, odom/keyframe rates, SC candidates, Quatro/Nano results, loop factors, graph status, TF owner/continuity, Bundle verification, Localization states, Relocalization recovery, CPU, RAM, and latency.
- [ ] **Step 2: Add RED absence behavior**: without required hardware Topics the script exits nonzero and writes `JT128 hardware validation pending`; it must never mark the run passed.
- [ ] **Step 3: Implement a portable runner** resolving project paths from its own location, accepting output directory/duration/threshold overrides, preserving raw command evidence, and writing a timestamped YAML/Markdown summary.
- [ ] **Step 4: Run GREEN** against fixtures only; record that real execution remains pending.

### Task 5: Final offline audit and Stage 9 checkpoint

**Interfaces:**
- Produces: the final offline verification record without converting pending hardware checks into pass.

- [ ] **Step 1: Run a requirement-by-requirement audit** against the design, all eight plans, package graph, Topics/TF/QoS, Map Bundle, state machine, relocalizer, and validation script.
- [ ] **Step 2: Run final verification**:

```bash
./scripts/build.sh
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon test --event-handlers console_direct+
colcon test-result --verbose
./tests/run_tests.sh
./scripts/run_jt128_full_validation.sh --dry-run
git diff --check
git diff main...HEAD -- src/FAST_LIO_Hesai src/a2w_fastlio2_bringup/config/jt128.yaml maps
git submodule status --recursive
```

Expected: all offline tests pass; dry-run enumerates every hardware check without claiming it ran; protected frontend commits remain unchanged.

- [ ] **Step 3: Update documentation** with the exact final state:

```text
Offline implementation and verification complete; JT128 hardware validation pending.
```

- [ ] **Step 4: Commit and push**:

```bash
git add src scripts tests docs README.md
git commit -m "feat(localization): add global relocalization and validation runner"
git push origin feature/slam-localization-backend
```

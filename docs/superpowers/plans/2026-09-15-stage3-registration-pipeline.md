# Stage 3 Registration Pipeline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build shared temporary local maps and a validated Quatro-to-Nano-GICP coarse/fine registration pipeline.

**Architecture:** Common abstract registration interfaces isolate Quatro and Nano-GICP. `LocalMapBuilder` creates equally scoped source and candidate maps; `MatchValidator` and `LoopValidator` accept implementation-neutral results.

**Tech Stack:** C++17, PCL, Eigen, OpenMP, TBB, TEASER++ v1.0, GoogleTest.

**Spec:** `docs/superpowers/specs/2026-09-15-a2w-slam-localization-backend-design.md`

## Global Constraints

- Reuse one common implementation for Mapping, Localization, and Relocalization.
- Pin Quatro `d27109b`, Nano-GICP `b21e79e`, TEASER++ `974574c` (first upstream Quatro API revision), and PMC `a2dfd61`.
- Do not import catkin or ROS 1 glue and do not change algorithm mathematics without a reproducing test.
- Do not implement a persistent Submap Manager.

---

### Task 1: Registration abstractions and neutral results

**Files:**
- Create: `src/a2w_fastlio_common/include/a2w_fastlio_common/registration.hpp`
- Modify: `src/a2w_fastlio_common/include/a2w_fastlio_common/types.hpp`
- Test: `src/a2w_fastlio_common/test/test_registration_interfaces.cpp`

**Interfaces:**
- Produces: `RegistrationResult`, `CoarseRegistration::align()`, and `FineRegistration::align()`.

- [x] **Step 1: Write RED tests** for default failure result, finite transform validation, rejection reason preservation, and fake `CoarseRegistration`/`FineRegistration` implementations callable only through interfaces.
- [x] **Step 2: Run RED**; expect missing types.
- [x] **Step 3: Implement the exact neutral API**; `success` requires a finite rigid transform and explicit convergence.

```cpp
struct RegistrationResult {
  bool success{false};
  bool converged{false};
  Pose3d transform{};
  double fitness{std::numeric_limits<double>::infinity()};
  double overlap{0.0};
  std::size_t correspondence_count{0};
  double elapsed_ms{0.0};
  std::string rejection_reason{};
};
virtual RegistrationResult align(
  const CloudConstPtr &, const CloudConstPtr &,
  const std::optional<Pose3d> &) const = 0;
```
- [x] **Step 4: Run GREEN** for the common suite.

### Task 2: Temporary LocalMapBuilder

**Files:**
- Create: `src/a2w_fastlio_common/include/a2w_fastlio_common/local_map_builder.hpp`
- Create: `src/a2w_fastlio_common/src/local_map_builder.cpp`
- Test: `src/a2w_fastlio_common/test/test_local_map_builder.cpp`

**Interfaces:**
- Consumes: `KeyFrameProvider::get(id)` and `LocalMapConfig`.
- Produces: `LocalMapResult build(center_id, before, after, provider)`.

- [x] **Step 1: Write RED tests** proving neighborhood bounds, pose application, deterministic voxel filtering, maximum-point enforcement, missing-ID errors, and equal source/target neighborhood policy.
- [x] **Step 2: Run RED**; expect missing builder.
- [x] **Step 3: Implement the provider/builder contract**; transform each body cloud by its supplied pose, then voxel-filter and deterministically cap points.

```cpp
class KeyFrameProvider {
public:
  virtual ~KeyFrameProvider() = default;
  virtual std::optional<KeyFrame> get(std::uint64_t id) const = 0;
};
LocalMapResult build(
  std::uint64_t center_id, std::size_t before, std::size_t after,
  const KeyFrameProvider & provider) const;
```
- [x] **Step 4: Run GREEN** and verify no ROS headers enter the target.

### Task 3: Curated registration dependencies and adapters

**Files:**
- Create: `third_party/{Quatro,nano_gicp}/` curated source, license, CMake, and provenance files
- Add submodules: `third_party/TEASER-plusplus` at `974574c` and `third_party/pmc` at `a2dfd61`
- Create: `src/a2w_fastlio_common/include/a2w_fastlio_common/{quatro_registration.hpp,nano_gicp_registration.hpp}`
- Create: `src/a2w_fastlio_common/src/{quatro_registration.cpp,nano_gicp_registration.cpp}`
- Test: `src/a2w_fastlio_common/test/test_registration_synthetic.cpp`

**Interfaces:**
- Implements: `QuatroRegistration final : CoarseRegistration` and
  `NanoGicpRegistration final : FineRegistration`.
- Keeps all TEASER++, Quatro, and Nano-GICP types private to `.cpp` or private implementation.

- [x] **Step 1: Write RED synthetic tests** using an asymmetric 3-D cloud transformed by known translation/yaw; assert Quatro produces a finite coarse transform and Nano-GICP reduces pose error and fitness when given the coarse guess. Add empty, too-small, NaN, and degenerate-line cases.
- [x] **Step 2: Run RED**; expect missing concrete adapters.
- [x] **Step 3: Import only used core files**, retain license headers, add `UPSTREAM.md`, and build private non-catkin targets. Compile TEASER++ `registration.cc`/`graph.cc` with fixed PMC; exclude TEASER++ I/O, tests, bindings, and its duplicate FPFH/matcher sources because Quatro provides those implementations.
- [x] **Step 4: Implement adapters** that validate data and parameters, measure elapsed time, translate upstream results to `RegistrationResult`, and expose no upstream type in public signatures.
- [x] **Step 5: Run GREEN**. Use deterministic seeds and tolerances wide enough for x86_64 CI but strict enough to reject the identity result.

### Task 4: Validators and coarse-to-fine pipeline

**Files:**
- Create: `src/a2w_fastlio_common/include/a2w_fastlio_common/{match_validator.hpp,registration_pipeline.hpp}`
- Create: `src/a2w_fastlio_common/src/{match_validator.cpp,registration_pipeline.cpp}`
- Test: `src/a2w_fastlio_common/test/test_match_validator.cpp`
- Test: `src/a2w_fastlio_common/test/test_registration_pipeline.cpp`

**Interfaces:**
- Consumes: abstract coarse/fine registrars and `MatchValidator`.
- Produces: `PipelineResult` containing coarse, fine, validation, and final transform records.

- [x] **Step 1: Write RED tests** for convergence, maximum fitness, minimum overlap/correspondences, maximum translation/rotation jump, candidate separation, coarse failure short-circuit, fine initial-guess handoff, and explicit rejection strings.
- [x] **Step 2: Run RED**; expect missing validators/pipeline.
- [x] **Step 3: Implement `RegistrationPipeline` using only interfaces** and a validator. Compute overlap/correspondences with a shared nearest-neighbor metric so Quatro and Nano-GICP report comparable evidence.

```cpp
PipelineResult run(
  const CloudConstPtr & source, const CloudConstPtr & target,
  const std::optional<Pose3d> & yaw_hint) const;
ValidationResult validate(
  const RegistrationResult & result,
  const ValidationContext & context) const;
```
- [x] **Step 4: Run GREEN**, including a false repeated-structure candidate that converges but is rejected by ambiguity or overlap.

### Task 5: Stage 3 checkpoint

**Files:**
- Create: `src/a2w_fastlio_mapping/config/{registration.yaml,loop_validation.yaml}`
- Modify: `README.md`, `third_party/THIRD_PARTY_NOTICES.md`, dependency bootstrap scripts

**Interfaces:**
- Produces: installed common algorithm targets and validated YAML parameters consumed by later Stages.

- [x] **Step 1: Add RED contract tests** for abstract-only upper-layer includes and complete YAML coverage.
- [x] **Step 2: Add validated offline defaults** and dependency installation/build instructions without describing them as JT128 tuned.
- [x] **Step 3: Run the full verification commands from Stage 2 Task 4.**
- [x] **Step 4: Commit and push**:

```bash
git add .gitmodules third_party src/a2w_fastlio_common src/a2w_fastlio_mapping/config scripts tests README.md
git commit -m "feat(common): add shared coarse-to-fine registration"
git push origin feature/slam-localization-backend
```

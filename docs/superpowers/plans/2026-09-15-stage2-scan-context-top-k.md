# Stage 2 Scan Context and Top-K Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a ROS-independent, parameterized place-recognition interface backed by Scan Context and an in-memory Top-K descriptor index.

**Architecture:** `a2w_fastlio_common` owns abstract `PlaceRecognition` and `DescriptorIndex` APIs. A curated Scan Context snapshot implements descriptor generation and yaw-aware distance; Mapping and future Localization consume only abstract types.

**Tech Stack:** C++17, Eigen 3.4, PCL 1.12, GoogleTest, ament_cmake.

**Spec:** `docs/superpowers/specs/2026-09-15-a2w-slam-localization-backend-design.md`

## Global Constraints

- Do not modify `src/FAST_LIO_Hesai`, `src/a2w_fastlio2_bringup/config/jt128.yaml`, or `maps`.
- Import no ROS 1 node, catkin, bag, launch, FAST-LIO, or Livox code.
- Pin Scan Context to `c8ef5b496a159cdfd7fa4761121178f25cd0a6bb` and preserve CC BY-NC-SA 4.0 notices.
- Production algorithms in `a2w_fastlio_common` must not include ROS message headers.
- Use test-first development and end Stage 2 with one independent commit and push.

---

### Task 1: Common place-recognition types and interfaces

**Files:**
- Create: `src/a2w_fastlio_common/include/a2w_fastlio_common/place_recognition.hpp`
- Modify: `src/a2w_fastlio_common/include/a2w_fastlio_common/types.hpp`
- Test: `src/a2w_fastlio_common/test/test_place_recognition_types.cpp`
- Modify: `src/a2w_fastlio_common/CMakeLists.txt`

**Interfaces:**
- Produces: `ScanDescriptor`, `LoopCandidate`, `CandidateFilter`, `PlaceRecognition`, and `DescriptorIndex`.

- [ ] **Step 1: Write failing interface tests**

```cpp
TEST(PlaceRecognitionTypes, CandidateCarriesRankDistanceAndYaw) {
  LoopCandidate c{42, 2, 0.12, 0.5};
  EXPECT_EQ(c.keyframe_id, 42U);
  EXPECT_EQ(c.rank, 2U);
  EXPECT_DOUBLE_EQ(c.distance, 0.12);
  EXPECT_DOUBLE_EQ(c.yaw_hint_rad, 0.5);
}

TEST(PlaceRecognitionTypes, RejectsInvalidDescriptorShape) {
  EXPECT_THROW(ScanDescriptor(0, 60, {}), std::invalid_argument);
}
```

- [ ] **Step 2: Run RED**

```bash
colcon test --packages-select a2w_fastlio_common --event-handlers console_direct+
```

Expected: compilation fails because `place_recognition.hpp` and its types do not exist.

- [ ] **Step 3: Implement the minimal interfaces**

```cpp
struct CandidateFilter { std::uint64_t max_inclusive_id; std::size_t exclude_recent; };
class PlaceRecognition {
public:
  virtual ~PlaceRecognition() = default;
  virtual ScanDescriptor describe(const CloudConstPtr &) const = 0;
};
class DescriptorIndex {
public:
  virtual ~DescriptorIndex() = default;
  virtual void add(std::uint64_t, const ScanDescriptor &) = 0;
  virtual std::vector<LoopCandidate> queryTopK(
    const ScanDescriptor &, std::size_t, const CandidateFilter &) const = 0;
};
```

- [ ] **Step 4: Run GREEN** using the Step 2 command; expect all common tests to pass.

### Task 2: Curated Scan Context core and provenance

**Files:**
- Create: `third_party/scancontext_tro/{LICENSE,UPSTREAM.md,CMakeLists.txt}`
- Create: `third_party/scancontext_tro/include/scancontext_tro/{scan_context.hpp,kdtree_vector_of_vectors.hpp,nanoflann.hpp}`
- Create: `third_party/scancontext_tro/src/scan_context.cpp`
- Create: `third_party/THIRD_PARTY_NOTICES.md`
- Test: `src/a2w_fastlio_common/test/test_scan_context_descriptor.cpp`

**Interfaces:**
- Consumes: `CloudConstPtr` and `ScanContextConfig`.
- Produces: deterministic `ScanDescriptor` with `rings * sectors` values plus ring and sector keys.

- [ ] **Step 1: Add failing descriptor tests** for empty clouds, invalid rings/sectors/radius, deterministic bin maxima, and a 90-degree synthetic rotation producing a quarter-turn yaw hint.
- [ ] **Step 2: Run RED** with `colcon test --packages-select a2w_fastlio_common`; expect missing `ScanContextPlaceRecognition`.
- [ ] **Step 3: Copy only the listed algorithm files from the pinned revision**, retain copyright/license, record SHA and local changes in `UPSTREAM.md`, remove the unused `pcl_conversions` include, namespace symbols, and parameterize geometry without changing descriptor equations.
- [ ] **Step 4: Implement `ScanContextPlaceRecognition final : public PlaceRecognition`** in `src/a2w_fastlio_common/{include/.../scan_context_place_recognition.hpp,src/scan_context_place_recognition.cpp}`.
- [ ] **Step 5: Run GREEN**; expect descriptor, yaw, empty-cloud, and invalid-configuration tests to pass.

### Task 3: Top-K descriptor index

**Files:**
- Create: `src/a2w_fastlio_common/include/a2w_fastlio_common/scan_context_index.hpp`
- Create: `src/a2w_fastlio_common/src/scan_context_index.cpp`
- Test: `src/a2w_fastlio_common/test/test_scan_context_index.cpp`

**Interfaces:**
- Produces: ordered, stable Top-K candidates with rank, distance, yaw, ID filtering, and recent-frame exclusion.

- [ ] **Step 1: Write failing tests** proving: results sort by distance then ID; `k > size` is safe; duplicates retain distinct IDs; recent IDs are excluded; repeated structures return multiple candidates rather than confirming Top-1; dimension mismatch throws without mutating the index.
- [ ] **Step 2: Run RED**; expect missing index implementation.
- [ ] **Step 3: Implement exact-distance Top-K first**, using ring-key preselection only when it preserves the tested result set. Reject `k == 0`, duplicate IDs, and mismatched shapes.
- [ ] **Step 4: Run GREEN** and the complete common package suite.

### Task 4: Stage 2 configuration, documentation, and checkpoint

**Files:**
- Create: `src/a2w_fastlio_mapping/config/scan_context.yaml`
- Modify: `README.md`
- Modify: `third_party/THIRD_PARTY_NOTICES.md`

**Interfaces:**
- Produces: installed place-recognition interfaces, Scan Context implementation target, provenance,
  and `scan_context.yaml` consumed by Mapping and later Localization.

- [ ] **Step 1: Add a failing repository test** in `tests/test_stage_contracts.py` that asserts every Scan Context parameter appears in YAML and no Mapping/Localization source includes concrete Scan Context headers.
- [ ] **Step 2: Run RED** with `./tests/run_tests.sh`; expect the contract test to fail.
- [ ] **Step 3: Add validated defaults** for rings 20, sectors 60, radius 80 m, sensor height 2 m, Top-K 5, recent exclusion 30, and candidate threshold 0.2. Document that these are offline defaults, not JT128-tuned values.
- [ ] **Step 4: Run full verification**:

```bash
./scripts/build.sh
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
./tests/run_tests.sh
git diff --check
git diff main...HEAD -- src/FAST_LIO_Hesai src/a2w_fastlio2_bringup/config/jt128.yaml maps
```

- [ ] **Step 5: Commit and push**:

```bash
git add third_party src/a2w_fastlio_common src/a2w_fastlio_mapping/config/scan_context.yaml tests README.md
git commit -m "feat(common): add Scan Context Top-K retrieval"
git push origin feature/slam-localization-backend
```

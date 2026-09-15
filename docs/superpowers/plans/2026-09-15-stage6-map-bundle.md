# Stage 6 Map Bundle V1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the shared `a2w_fastlio_map` package and a versioned, checksummed, atomic Map Bundle V1 format.

**Architecture:** The Map package owns keyframes, persistent descriptors, manifests, and all map I/O. Mapping writes bundles through this API; Localization loads the same API read-only.

**Tech Stack:** C++17, PCL PCD I/O, yaml-cpp, OpenSSL SHA-256, GoogleTest, ROS 2 service adapters.

**Spec:** `docs/superpowers/specs/2026-09-15-a2w-slam-localization-backend-design.md`

## Global Constraints

- `a2w_fastlio_map -> a2w_fastlio_common` is the only algorithm dependency; no Mapping/Localization dependency is allowed.
- Reject paths escaping the bundle root and never partially replace a valid bundle.
- Full `global_map.pcd` is file-based, not a persistent DDS payload.

---

### Task 1: Package boundary and in-memory databases

**Files:**
- Create: `src/a2w_fastlio_map/{CMakeLists.txt,package.xml}`
- Create: `src/a2w_fastlio_map/include/a2w_fastlio_map/{keyframe_database.hpp,descriptor_database.hpp,map_manager.hpp}`
- Create: corresponding `src/*.cpp`
- Test: `src/a2w_fastlio_map/test/test_databases.cpp`

**Interfaces:**
- Implements: `KeyFrameProvider` and common `DescriptorIndex` without exposing mutable storage.
- Produces: `MapSnapshot MapManager::snapshot() const`.

- [x] **Step 1: Write RED tests** for ordered IDs, duplicate rejection, immutable read snapshots, keyframe/descriptor count consistency, Top-K delegation through the common abstract API, and read-only mode mutation rejection.
- [x] **Step 2: Run RED**; expect package/types missing.
- [x] **Step 3: Implement databases and manager** without ROS messages. Store owned clouds and return const snapshots.
- [x] **Step 4: Run GREEN** and a dependency-graph test proving the package has no Mapping/Localization dependency.

### Task 2: Manifest and safe MapIO

**Files:**
- Create: `src/a2w_fastlio_map/include/a2w_fastlio_map/{map_manifest.hpp,map_io.hpp}`
- Create: corresponding `src/*.cpp`
- Test: `src/a2w_fastlio_map/test/test_map_manifest.cpp`

**Interfaces:**
- Produces: `MapManifest::build(root, files)`, `verify(root)`, and confined relative-path helpers.

- [x] **Step 1: Write RED tests** using temporary directories for stable SHA-256, modified-file detection, missing-file detection, duplicate manifest entries, absolute/traversal path rejection, and deterministic ordering.
- [x] **Step 2: Run RED**; expect missing manifest API.
- [x] **Step 3: Implement binary-safe streaming SHA-256** and canonical relative-path confinement. Symlinks resolving outside the bundle root are rejected.

```cpp
std::string sha256File(const std::filesystem::path & file);
std::filesystem::path confinedPath(
  const std::filesystem::path & root, const std::filesystem::path & relative);
ManifestVerification verifyManifest(const std::filesystem::path & root) const;
```
- [x] **Step 4: Run GREEN** including filenames containing spaces.

### Task 3: Writer/Reader round trip and atomic replacement

**Files:**
- Create: `src/a2w_fastlio_map/include/a2w_fastlio_map/{map_bundle_writer.hpp,map_bundle_reader.hpp,map_bundle.hpp}`
- Create: corresponding `src/*.cpp`
- Test: `src/a2w_fastlio_map/test/test_map_bundle_round_trip.cpp`
- Test: `src/a2w_fastlio_map/test/test_map_bundle_failures.cpp`

**Interfaces:**
- Consumes: `MapBundleData` containing metadata, keyframes, poses, descriptors, full map, and configs.
- Produces: `BundleWriteResult write(path,data)` and `MapBundle read(path,ReadMode::kReadOnly)`.

- [x] **Step 1: Write RED round-trip tests** for metadata, exact IDs/stamps/poses, PCD point data, descriptors, full map, config snapshots, dependency revisions, and pending hardware status.
- [x] **Step 2: Write RED failure tests** for wrong schema, corrupt hash, missing cloud, dimension/count mismatch, NaN pose, unreadable PCD, staging failure, replacement rollback, and stale staging directories.
- [x] **Step 3: Implement Bundle V1** exactly as the design layout. Use unique sibling staging, production-reader self-validation, destination-to-backup rename, final rename, rollback, and cleanup only of the operation's own staging path.
- [x] **Step 4: Run GREEN** and verify an existing valid destination survives every injected failure.

### Task 4: ROS save service and Stage 6 checkpoint

**Files:**
- Create: `src/a2w_fastlio_msgs/srv/SaveMapBundle.srv`
- Create: `src/a2w_fastlio_map/config/map.yaml`
- Create: `src/a2w_fastlio_mapping/include/a2w_fastlio_mapping/map_bundle_service.hpp`
- Create: `src/a2w_fastlio_mapping/src/map_bundle_service.cpp`
- Test: `src/a2w_fastlio_mapping/test/test_map_bundle_service_launch.py`
- Modify: Mapping node, README, scripts

**Interfaces:**
- Provides: `/mapping/save_map_bundle` using the exact service below.

```text
string output_path
---
bool success
string message
string bundle_uuid
uint64 keyframe_count
string resolved_path
```

- [x] **Step 1: Write RED launch tests** for successful save, busy/reentrant rejection, invalid path, reported UUID/count/path, and no partial bundle on shutdown.
- [x] **Step 2: Implement the Mapping-side ROS service adapter** with configurable bundle root and bounded worker queue. The callback takes an immutable Mapping snapshot and calls
  `a2w_fastlio_map::MapBundleWriter`; `a2w_fastlio_map` remains ROS-message independent and does not
  depend on Mapping or messages. The service never subscribes to preview map as source data.
- [x] **Step 3: Run full build/tests and a CLI load/verify command against a generated temporary bundle.**
- [ ] **Step 4: Commit and push**:

```bash
git add src/a2w_fastlio_map src/a2w_fastlio_msgs src/a2w_fastlio_mapping tests scripts README.md
git commit -m "feat(map): add versioned Map Bundle V1"
git push origin feature/slam-localization-backend
```

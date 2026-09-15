# A2W FAST-LIO Mapping, Localization, and Global Relocalization Design

**Date:** 2026-09-15

**Status:** Approved for offline implementation. Runtime validation against the A2W and Hesai
JT128 remains a separate final gate.

## 1. Goal and completion language

Build a ROS 2 Humble backend around the existing Hesai FAST-LIO frontend:

```text
FAST_LIO_Hesai
  -> SC-QN Mapping
  -> Map Bundle V1
  -> Localization-QN
  -> Global Relocalization
```

Stages 2 through 9 may be implemented and verified without the robot. Offline completion must be
reported exactly as:

```text
Offline implementation and verification complete; JT128 hardware validation pending.
```

No offline test, synthetic point cloud, historical PCD, launch test, or code inspection is evidence
that JT128 Topic frequency, QoS compatibility, frames, timestamps, or real-time performance have
been validated on hardware.

## 2. Immutable frontend boundary

`src/FAST_LIO_Hesai` is the only LIO frontend. This work does not download another FAST-LIO,
replace that submodule, or modify its JT128 driver integration, preprocessing, ESIKF, ikd-tree, or
stable mapping behavior. The protected baseline is:

```text
FAST_LIO_Hesai: 16e97cdcc4260b9ba6241518a19ad8384224ba48
ikd-tree:       e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4
```

The following paths are protected by a diff check at every Stage:

```text
src/FAST_LIO_Hesai
src/a2w_fastlio2_bringup/config/jt128.yaml
maps
```

The confirmed code-level frontend contract remains:

```text
/Odometry                nav_msgs/msg/Odometry
/cloud_registered_body   sensor_msgs/msg/PointCloud2
camera_init -> body
```

Until hardware headers and the complete robot TF tree are captured, the backend publishes
`map -> camera_init`. It must not rename this chain to `map -> odom -> base_link` by assumption.

## 3. Package tree and acyclic dependencies

```text
A2W_FastLio2/
├── third_party/
│   ├── scancontext_tro/
│   ├── Quatro/
│   ├── nano_gicp/
│   ├── TEASER-plusplus/
│   ├── pmc/
│   └── THIRD_PARTY_NOTICES.md
├── src/
│   ├── FAST_LIO_Hesai/
│   ├── a2w_fastlio_common/
│   ├── a2w_fastlio_mapping/
│   ├── a2w_fastlio_map/
│   ├── a2w_fastlio_localization/
│   ├── a2w_fastlio_msgs/
│   └── a2w_fastlio2_bringup/
├── scripts/
└── docs/
```

The dependency graph is mandatory. Each arrow below means "consumer depends on dependency":

```text
a2w_fastlio_map          -> a2w_fastlio_common
a2w_fastlio_mapping      -> a2w_fastlio_common + a2w_fastlio_map + a2w_fastlio_msgs
a2w_fastlio_localization -> a2w_fastlio_common + a2w_fastlio_map + a2w_fastlio_msgs
a2w_fastlio2_bringup     -> a2w_fastlio_mapping + a2w_fastlio_localization
```

More precisely:

- `a2w_fastlio_common` does not depend on Mapping, Map, or Localization. Its algorithm library is
  ROS-message independent.
- `a2w_fastlio_map` depends on `common`. It owns all persistent map and Map Bundle behavior.
- `a2w_fastlio_mapping` depends on `common + map + msgs`.
- `a2w_fastlio_localization` depends on `common + map + msgs`.
- `a2w_fastlio2_bringup` contains no algorithms or map I/O; it only composes launch files and
  configuration profiles.
- No package dependency cycle is permitted. A repository test topologically checks this graph.

`a2w_fastlio_msgs` contains ROS interface definitions and therefore remains separate from the
ROS-independent algorithms. Common result types used inside algorithms are plain C++ structs;
Mapping and Localization nodes adapt them to ROS messages at their boundary.

## 4. Common algorithm abstraction boundary

Mapping, Localization, and Global Relocalization depend on interfaces, not concrete algorithm
classes:

```cpp
class PlaceRecognition {
public:
  virtual ~PlaceRecognition() = default;
  virtual ScanDescriptor describe(const CloudConstPtr & cloud) const = 0;
  virtual std::vector<LoopCandidate> queryTopK(
    const ScanDescriptor & query, std::size_t k,
    const CandidateFilter & filter) const = 0;
};

class CoarseRegistration {
public:
  virtual ~CoarseRegistration() = default;
  virtual RegistrationResult align(
    const CloudConstPtr & source, const CloudConstPtr & target,
    const std::optional<Pose3d> & initial_guess) const = 0;
};

class FineRegistration {
public:
  virtual ~FineRegistration() = default;
  virtual RegistrationResult align(
    const CloudConstPtr & source, const CloudConstPtr & target,
    const Pose3d & initial_guess) const = 0;
};
```

The production implementations are:

```text
PlaceRecognition   -> ScanContextPlaceRecognition
CoarseRegistration -> QuatroRegistration
FineRegistration   -> NanoGicpRegistration
```

`RegistrationResult` is implementation-neutral and includes:

```text
success
transform
converged
fitness
overlap
correspondence_count
elapsed_ms
rejection_reason
```

`LoopCandidate` includes keyframe ID, Scan Context distance, yaw hint, and rank. Higher layers may
not include or name upstream concrete headers. This preserves the ability to replace Scan Context,
Quatro, or Nano-GICP without rewriting the system framework.

`a2w_fastlio_common` also owns `LocalMapBuilder`, pose/point-cloud utilities, validators, and the
abstract descriptor query API. Persistent descriptors and keyframe data are implemented by
`a2w_fastlio_map` behind that API.

## 5. Third-party source policy

The project reuses mature algorithm cores while implementing its ROS 2 structure itself. It does
not import upstream ROS 1 nodes, `ros::NodeHandle`, publishers/subscribers, catkin glue, ROS 1 bag
I/O, launch files, FAST-LIO, Livox drivers, Topic glue, or TF glue.

| Module | Upstream | Fixed revision | Reused files/components | Project changes | Reason | License | Wrapper |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Scan Context | `engcang/scancontext_tro` | `c8ef5b496a159cdfd7fa4761121178f25cd0a6bb` | `Scancontext.cpp/.h`, KD-tree adapter, nanoflann | Remove unused ROS include, add namespace/config constructor, const-correct API; equations unchanged | ROS-free build, parameterized rings/sectors/radius/height and Top-K | CC BY-NC-SA 4.0 | `ScanContextPlaceRecognition` |
| Quatro | `engcang/Quatro` | `d27109bd6a1798e9cf2e0c2a4daac6af16e7bc23` | FPFH, matcher, Quatro module core | New non-catkin CMake adapter; source changes only for demonstrated compiler compatibility | Build in ROS 2 workspace without importing ROS 1 packaging | `License` GPL-3.0; README/package metadata also claims CC BY-NC-SA 4.0 | `QuatroRegistration` |
| Nano-GICP | `engcang/nano_gicp` | `b21e79edcceb7c6e86ec0dfec90f451b796b683a` | Nano-GICP, LSQ, GICP, nanoflann core | New non-catkin CMake adapter; no planned math changes | Build one shared fine-registration implementation | MIT plus retained BSD-3-Clause notices | `NanoGicpRegistration` |
| TEASER++ | `MIT-SPARK/TEASER-plusplus` | `974574c2fe8d8523e9f7d7be0500c427ba1d1f9a` | `registration.cc`, `graph.cc`, and public solver headers | Forced compatibility include supplies the legacy unqualified `vector`; pinned source stays unchanged; private CMake target excludes I/O, tests, bindings, and duplicate FPFH/matcher implementations | First upstream revision that adds the Quatro solver API required by the pinned Quatro core | MIT | Private dependency of Quatro target |
| PMC | `jingnanshi/pmc` | `a2dfd612a501bca83c47206255dbbff619481f97` | Maximum-clique library sources and headers | No source change; custom private CMake target omits CLI/test programs | Fixed transitive dependency of TEASER++ registration | GPL-3.0-or-later | Private dependency of TEASER++ target |
| GTSAM | Ubuntu system package | `4.1.1` | GTSAM and iSAM2 public API | None; not vendored | Already available and avoids a second GTSAM build | BSD-3-Clause | `PoseGraphOptimizer` |

Scan Context, Quatro, and Nano-GICP snapshots contain only used algorithm files, their original
license, and an `UPSTREAM.md` recording source revision and local compatibility patches. TEASER++
and PMC are pinned non-ROS submodules because selectively copying their transitive solver
implementations would make provenance and maintenance worse. Quatro's own FPFH and matcher are
compiled exactly once; the same-named TEASER++ feature sources are intentionally excluded. GTSAM
remains a system dependency.

## 6. Mapping data flow

```text
/Odometry + /cloud_registered_body
  -> exact timestamp FrontendMessageAdapter
  -> KeyFrameManager
  -> PlaceRecognition::describe/queryTopK
  -> LoopCandidateManager
  -> LocalMapBuilder(current neighborhood, candidate neighborhood)
  -> CoarseRegistration::align
  -> FineRegistration::align
  -> LoopValidator
  -> PoseGraphOptimizer(Prior/Odometry/Loop factors, iSAM2)
  -> optimized keyframe poses
  -> MapOdomManager
  -> map -> camera_init
  -> OptimizedMapBuilder
  -> /mapping/optimized_map_preview
```

Scan Context only proposes Top-K candidates. It never confirms a loop. Candidate filtering excludes
recent frames but cannot use corrected spatial distance as a mandatory global-relocalization gate.

`LocalMapBuilder` combines configurable neighboring keyframes into temporary source and target
maps. There is no persistent Submap Manager in this design. Its data-source interface allows a
future Submap Manager without changing place recognition, registration, PGO, or Localization APIs.

Quatro produces the coarse transform. Nano-GICP consumes that transform as its initial guess. The
validator requires configured geometric evidence; `hasConverged()` alone is insufficient.

Pose graph optimization changes backend keyframe poses only. It never writes corrections back into
FAST-LIO's ESIKF or ikd-tree.

The synchronized input callback performs only validation, conversion, and bounded-queue insertion.
Descriptor generation, local-map construction, registration, and PGO run in workers so the frontend
callback cannot accumulate an unbounded backlog.

## 7. Optimized map preview and full map ownership

The backend does not retain a complete high-density map in DDS transient-local history.

```text
/mapping/optimized_map_preview
```

is a downsampled, bounded-size RViz/debug product. Its voxel size, maximum points, publication
period, QoS, and enable flag are parameters. Default QoS is reliable, transient-local, keep-last 1.

The complete optimized map exists only through:

```text
a2w_fastlio_map -> Map Bundle V1 -> global_map.pcd
```

`OptimizedMapBuilder` can stream keyframes through a voxel accumulator for preview or full-map
generation. Mapping does not implement a separate global-map writer, and Localization never treats
the preview Topic as the authoritative map.

## 8. Localization data flow

```text
MapBundleReader
  -> KeyFrameDatabase + DescriptorDatabase + optimized poses

/Odometry + /cloud_registered_body
  -> FrontendMessageAdapter
  -> LocalizationManager
      ├── high-rate global pose from last valid T_map_camera_init
      └── low-rate map matching
            -> LocalMapSelector
            -> LocalMapBuilder(nearby keyframes)
            -> CoarseRegistration
            -> FineRegistration
            -> MatchValidator
            -> update T_map_camera_init
```

Localization opens a Map Bundle read-only. It does not append keyframes, factors, descriptors, or
map points. Match failure does not silently refresh correction age. A recently valid correction may
support continuous output while the monitor reports its age and degraded state.

## 9. Global relocalization data flow

```text
LocalizationMonitor -> LOST -> RELOCALIZING
  -> current temporary local map
  -> PlaceRecognition full-map Top-K
  -> for every candidate:
       candidate LocalMapBuilder
       -> CoarseRegistration
       -> FineRegistration
       -> MatchValidator
  -> rank validated candidates
  -> ambiguity margin and multi-frame consistency
  -> restore map -> camera_init
  -> LOCALIZED
```

Scan Context Top-1 can never directly establish a position. Repeated factory structures are handled
by validating multiple candidates, comparing the best and second-best results, and requiring either
one configurable strong result or consistent evidence across configurable consecutive frames.

The state sequence is:

```text
INITIALIZING -> LOCALIZED -> DEGRADED -> LOST -> RELOCALIZING -> LOCALIZED
```

All transitions use explicit counters, timeouts, and hysteresis. A failed relocalization attempt
remains observable and does not publish a newly valid global correction.

The Stage 8 transition contract is:

| Current state | Evidence or command | Next state | Global-output rule |
| --- | --- | --- | --- |
| `INITIALIZING` | configured consecutive accepted normal matches | `LOCALIZED` | Enable only after the threshold is met |
| `INITIALIZING` | rejected/no match | `INITIALIZING` | Disabled |
| `LOCALIZED` | fewer than `degraded_failures_required` failures | `LOCALIZED` | Keep the last non-stale correction |
| `LOCALIZED` | configured consecutive failures | `DEGRADED` | Keep the last non-stale correction |
| `DEGRADED` | configured consecutive normal successes | `LOCALIZED` | Continue with recovered correction |
| `LOCALIZED` or `DEGRADED` | failure count reaches `lost_failures_required`, or correction age reaches its boundary | `LOST` | Disable pose/odom/path and TF based on the old correction |
| `LOST` | ordinary local match, accepted or rejected | `LOST` | Disabled; no direct recovery is permitted |
| `LOST` | explicit global-relocalization start | `RELOCALIZING` | Disabled |
| `RELOCALIZING` | rejected/ambiguous evidence | `RELOCALIZING` | Disabled and observable |
| `RELOCALIZING` | strong single result or configured consecutive accepted global results | `LOCALIZED` | Enable the newly validated correction |
| `RELOCALIZING` | timeout boundary reached | `LOST` | Disabled |
| any active state | explicit reset | `INITIALIZING` | Disabled |

Every boundary in this table is inclusive and driven by monotonic data timestamps. Parameters are
declared under `monitor.*` in `a2w_fastlio_localization/config/relocalization.yaml`: initialization,
degradation, loss, normal-recovery and relocalization counts; correction staleness and relocalization
timeout; and strong-result fitness, overlap and correspondence thresholds. `/localization/status`
publishes the state, transition reason, counters, correction age, candidate ID and registration
metrics. Until the final JT128 procedure succeeds, its `hardware_validation_pending` field remains
true.

## 10. Topic, frame, timestamp, and QoS contract

Every Topic name, frame name, QoS policy, depth, queue size, worker count, path, and algorithm
threshold is a declared ROS parameter. Invalid values fail node startup.

Default input contract:

| Topic | Type | Frame | Timestamp | Default QoS |
| --- | --- | --- | --- | --- |
| `/Odometry` | `nav_msgs/msg/Odometry` | parent `camera_init`, child `body` | FAST-LIO lidar end time | reliable, volatile, keep-last 20 |
| `/cloud_registered_body` | `sensor_msgs/msg/PointCloud2` | `body` | exactly the same lidar end time | reliable, volatile, keep-last 20 |

Stage 1 continues to reject a one-nanosecond mismatch. Runtime QoS compatibility is not considered
hardware-validated until the final JT128 test.

Mapping outputs:

```text
/mapping/keyframe_odom
/mapping/keyframe_cloud
/mapping/loop_candidates
/mapping/registration_status
/mapping/optimized_odom
/mapping/optimized_path
/mapping/optimized_map_preview
/mapping/loop_markers
```

Localization outputs:

```text
/localization/pose
/localization/odom
/localization/path
/localization/status
/localization/current_cloud
/localization/local_map
/localization/registration_status
```

High-rate pose, odom, and status default to reliable/volatile/keep-last 10 or 20. Debug clouds
default to best-effort/volatile/keep-last 1. Preview map and path default to
reliable/transient-local/keep-last 1. ROS TF uses the standard `/tf` QoS.

Mapping and Localization are separate launch modes:

```text
mapping.launch.py
localization.launch.py
```

Each mode contains exactly one project global-TF owner. A reliable transient-local ownership
announcement identifies mode and node. Before activation, the owner observes a configurable
conflict window; if it sees a foreign active owner it refuses to publish. If a conflict appears at
runtime, it stops global TF and publishes a fault. This is an application-level startup/runtime
check, not a required host filesystem lock. Launch tests verify that each profile contains one owner
and never starts Mapping and Localization backends together.

## 11. YAML organization

```text
src/a2w_fastlio_mapping/config/
├── keyframe.yaml
├── scan_context.yaml
├── registration.yaml
├── loop_validation.yaml
├── pose_graph.yaml
└── mapping_topics.yaml

src/a2w_fastlio_map/config/
└── map.yaml

src/a2w_fastlio_localization/config/
├── localization.yaml
├── map_matching.yaml
├── relocalization.yaml
└── localization_topics.yaml

src/a2w_fastlio2_bringup/config/profiles/
├── jt128_mapping.yaml
├── jt128_localization.yaml
└── offline_test.yaml
```

The files cover input/output Topics, frames, all QoS fields, exact-sync and worker queues, thread
counts, keyframe thresholds, Scan Context geometry and Top-K, Local Map neighborhoods and voxel
filters, Quatro/TEASER++ parameters, Nano-GICP parameters, validator thresholds, GTSAM noise models
and relinearization, preview limits, Map Bundle paths, matching rates, and state-machine gates.

Bringup selects and composes profiles. It does not duplicate package defaults or implement parsing.

## 12. Map package and Map Bundle V1

`a2w_fastlio_map` owns:

```text
MapManager
MapBundleWriter
MapBundleReader
KeyFrameDatabase
DescriptorDatabase
MapManifest
MapIO
```

Mapping supplies keyframes, optimized poses, descriptors, and the generated full map to this API.
Localization consumes the same API read-only.

Bundle layout:

```text
map_bundle/
├── metadata.yaml
├── manifest.sha256
├── optimized_poses.csv
├── keyframes/
│   ├── index.csv
│   └── clouds/000000.pcd ...
├── descriptors/scan_context.bin
├── global_map.pcd
└── config/
    ├── mapping_effective.yaml
    ├── scan_context.yaml
    └── registration.yaml
```

Metadata includes schema version, UUID, UTC creation time, frames, point type, keyframe and
descriptor counts, descriptor geometry, global-map leaf size, frontend/submodule revisions,
third-party revisions, GTSAM version, effective configuration hashes, creation status, and hardware
validation status.

Writer behavior:

1. Validate finite poses, nonempty required clouds, IDs, timestamps, descriptor dimensions, and
   equal keyframe/pose/descriptor counts.
2. Write into a unique sibling staging directory.
3. Generate SHA-256 hashes for every data/config file.
4. Read and validate the staging bundle with the production Reader.
5. Atomically rename the completed staging directory into place on the same filesystem. Existing
   destination replacement uses a recoverable backup rename and rollback on failure.

Reader behavior validates schema, required files, paths confined to the bundle root, hashes,
dimensions, IDs, frames, finite transforms, and point-cloud readability before exposing data.

Offline-created test bundles use:

```yaml
creation_status: offline_verified
hardware_validation_status: pending
```

This records software-format verification and is not a claim that a map came from JT128 hardware.

## 13. Stage 2 through 9

| Stage | Delivery | Required offline evidence |
| --- | --- | --- |
| 2 | Scan Context, abstraction interfaces, in-memory descriptor index, Top-K | Descriptor geometry, yaw, ranking, recent-frame exclusion, repeated-structure candidates, invalid configuration |
| 3 | LocalMapBuilder, Quatro, Nano-GICP, validators | Known rigid transforms, coarse-to-fine handoff, false candidates, empty/degenerate clouds, rejection reasons |
| 4 | GTSAM Prior/Odometry/Loop factors and iSAM2 | Synthetic graph, loop-error reduction, invalid factors, stable incremental updates |
| 5 | Optimized odom/path, `map -> camera_init`, preview map | Transform consistency, single owner/conflict, bounded preview, global-map reconstruction input |
| 6 | `a2w_fastlio_map` and Map Bundle V1 | Round trip, SHA-256 corruption, missing files, wrong schema, path confinement, atomic failure recovery |
| 7 | Normal Localization | Known-map matching, local selection, read-only database, low-rate correction/high-rate output |
| 8 | LocalizationMonitor | Every state edge, hysteresis, consecutive failures, stale correction, recovery gates |
| 9 | GlobalRelocalizer and hardware validation runner | Full-map Top-K, repeated false candidate, kidnapped-robot recovery, multi-frame confirmation, script dry-run |

Every Stage follows test-driven development and ends with:

```text
targeted unit tests
offline integration tests
full colcon build
full colcon test
repository portability/path tests
protected frontend diff check
one Stage commit
push feature/slam-localization-backend
```

No Stage depends on hardware to finish its offline implementation. Hardware validation is deferred
to `scripts/run_jt128_full_validation.sh`, which checks Topic frequency, QoS, frames, timestamps,
FAST-LIO odometry, keyframe rate, Scan Context, Quatro, Nano-GICP, loop validation, GTSAM, global TF,
Map Bundle, Localization, Relocalization, CPU, RAM, and latency in one recorded run.

## 14. Licensing and dependency risk

The repository must not be described as uniformly Apache-2.0 after restricted third-party code is
introduced. Original project files retain their own declared license, while every imported file
retains upstream copyright, license text, and provenance.

Known risks:

- Scan Context is CC BY-NC-SA 4.0 and restricts commercial use.
- Quatro has conflicting upstream license signals: a GPL-3.0 license file and CC BY-NC-SA wording
  in its README/package metadata. This must be treated conservatively until clarified by its owner
  or legal review.
- Directory and target isolation improve provenance but do not automatically remove license duties
  for linked or distributed software.
- Commercial delivery requires a separate license decision, upstream permission, or replacement of
  restricted components. This design does not assume the project is non-commercial.
- TEASER++ v1.0, Nano-GICP, and GTSAM have permissive licenses, but notices must still be retained.
- Fixed revisions and hashes prevent unreviewed upstream changes.

## 15. Explicit exclusions

- No second FAST-LIO frontend or Livox driver.
- No ROS 1 node, launch, bag, Topic, or TF glue.
- No persistent Submap Manager in Stages 2 through 9.
- No PGO correction written into FAST-LIO ESIKF or ikd-tree.
- No Nav2 integration in this scope.
- No robot, network, driver, lidar-target, or Unitree SLAM changes.
- No hardware-validation claim until the final script is executed while connected to the A2W/JT128.

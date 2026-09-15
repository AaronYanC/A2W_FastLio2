# Offline Verification Record

This record distinguishes implemented and tested software from evidence that can only be collected
with the Unitree A2-W Pro and Hesai JT128 connected.

## Delivered chain

```text
FAST_LIO_Hesai
  -> SC-QN Mapping (Top-K, Quatro, Nano-GICP, LoopValidator, GTSAM/iSAM2)
  -> Map Bundle V1 (global_map.pcd is the authoritative complete map)
  -> Localization-QN
  -> LOST / RELOCALIZING Global Relocalization
```

The frontend remains the pinned Hesai ROS2 implementation. No driver, ESIKF, ikd-tree, robot
network, lidar target, or Unitree SLAM service change is part of this delivery.

## Requirement audit

| Requirement | Offline evidence | Hardware status |
| --- | --- | --- |
| Acyclic package dependency graph | Contract test parses all package manifests | Not hardware-dependent |
| Shared algorithm abstraction boundary | Concrete-header guard and fake-interface registration tests | Pending runtime tuning |
| Scan Context Top-K and repeated structures | Descriptor/ranking tests and repeated-factory fixture | Pending JT128 scene validation |
| Quatro and Nano-GICP | Known-transform and false-candidate tests through one shared pipeline | Pending rate/latency validation |
| Loop validation and GTSAM/iSAM2 | Synthetic loop and incremental pose-graph tests | Pending live trajectory validation |
| Optimized outputs and single TF owner | ROS2 tests for QoS, frames, transform consistency, owner conflict | Pending live TF continuity |
| Map Bundle V1 | Round trip, corruption, traversal, rollback, read-only tests | Pending real map capture |
| Localization monitor | Generated event sequences, boundaries, stale correction, timeout | Pending live thresholds |
| Global relocalization | Top-K audit, ambiguity rejection, worker, kidnapped-pose ROS2 test | Pending factory tests |
| Consolidated validation | Dry-run and missing-hardware fixture remain pending/nonzero | Not run with hardware |

## Offline commands

```bash
./scripts/build.sh
source install/setup.bash
colcon test --executor sequential --return-code-on-test-failure
colcon test-result --verbose
./tests/run_tests.sh
./scripts/run_jt128_full_validation.sh --dry-run --mode all
```

ROS launch tests run sequentially because they intentionally exercise shared DDS/TF contracts.

## Deferred acceptance

With the robot available, start the requested Mapping or Localization mode and run
`scripts/run_jt128_full_validation.sh` to collect Topic frequencies, endpoint QoS, frames,
timestamps, odometry/keyframe rates, registration and loop evidence, graph output, TF continuity,
Map Bundle integrity, Localization/Relocalization transitions, CPU, RAM, and latency. A dry run is
only a checklist and never changes hardware status.

```text
Offline implementation and verification complete; JT128 hardware validation pending.
```

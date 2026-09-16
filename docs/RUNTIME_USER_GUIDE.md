# A2W JT128 FAST-LIO2 运行与使用手册

本文对应分支 `feature/slam-localization-backend`、基线提交 `6c07bd9`。命令、Topic、
Service、消息字段和参数均按当前仓库源码整理，不把设计目标当成现有接口。

> 当前状态：`Offline implementation and verification complete; JT128 hardware validation pending.`
>
> 在完成实机验收前，本文中的频率、阈值和实时表现只是离线安全初值，不代表已经通过
> A2W/JT128 验证。所有命令只操作 PC 端；不要修改机器人、雷达网络或驱动，也不要停止
> 宇树自带 SLAM 服务。

以下命令默认从仓库根目录执行。首次运行前先执行 `./scripts/bootstrap.sh` 和
`./scripts/build.sh`。Mapping、Localization 和 FAST-LIO-only 启动脚本都会自动加载本工作区、
设置 CycloneDDS，并解析仓库根目录。

每个用于执行 `ros2 topic/service/run` 的新终端应先加载相同环境。仓库已有的公共函数会
自动选择机器人子网网卡并生成与启动脚本相同的 CycloneDDS 配置：

```bash
repository_root=$(pwd -P)
source scripts/lib/a2w_common.sh
a2w_source_ros_environment "$repository_root"
a2w_prepare_cyclonedds "$repository_root"
```

后文的 `ros2 ...` 命令均假定当前终端已经执行过这段初始化。也可以提前显式设置
`A2W_NETWORK_INTERFACE`、`A2W_PC_IP` 和 `A2W_DDS_PEER` 后再执行。

## 1. 整体运行模式

当前有三个可启动的运行模式，以及一个由 Localization 内部自动进入的状态：

| 模式 | 启动入口 | 用途 | `map → camera_init` 发布者 |
| --- | --- | --- | --- |
| FAST-LIO only | `./scripts/run_fastlio_jt128_pc.sh` | 仅运行 Hesai FAST-LIO 前端 | 无 |
| Mapping | `./scripts/run_a2w_mapping.sh` | 关键帧、回环、PGO、优化地图和 Map Bundle | `mapping_backend_node` |
| Localization | `./scripts/run_a2w_localization.sh MAP_BUNDLE` | 加载 Map Bundle，持续全局定位 | `localization_node` |
| Global Relocalization | 无独立启动命令 | Localization 已进入 `LOST` 后自动执行 | `localization_node` 恢复成功后继续发布 |

Mapping 与 Localization 不能同时运行。二者都会启动自己的 FAST-LIO 前端，也都会声明
`map → camera_init` 所有权；同时启动会造成重复前端和全局 TF owner 冲突。Global
Relocalization 不是第四个进程，也不能与 Localization 分开运行。

推荐工作流：

```text
第一次进入新场地
→ Mapping
→ 完成路线并等待后端处理
→ 保存 Map Bundle
→ 检查 Map Bundle
→ 停止 Mapping

以后重新开机
→ Localization 加载已有 Map Bundle
→ 初始局部匹配成功并进入 LOCALIZED
→ 持续实时定位

已正常定位后发生异常
→ DEGRADED
→ LOST
→ RELOCALIZING
→ 自动全局重定位
→ LOCALIZED
```

注意：当前“冷启动初始定位”与“已定位后的丢失恢复”不是同一条路径，详见第 7 节。

## 2. FAST_LIO_Hesai 单独运行

启动：

```bash
./scripts/run_fastlio_jt128_pc.sh
```

不需要 RViz 时：

```bash
./scripts/run_fastlio_jt128_pc.sh rviz:=false
```

该脚本实际启动 `a2w_fastlio2_bringup/jt128_mapping.launch.py` 中的
`fast_lio/fastlio_mapping`。默认不保存前端 PCD。

另开终端检查输入和输出：

```bash
./scripts/check_lidar_topics.sh
ros2 topic hz /lidar_points
ros2 topic hz /lidar_imu
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
ros2 topic hz /cloud_registered_body
ros2 topic echo --once /Odometry
ros2 topic echo --once /cloud_registered_body
ros2 run tf2_ros tf2_echo camera_init body
```

真实接口含义：

| Topic / TF | 类型与坐标系 | 用途 |
| --- | --- | --- |
| `/lidar_points` | JT128 点云 | FAST-LIO 输入 |
| `/lidar_imu` | JT128 IMU | FAST-LIO 输入 |
| `/Odometry` | `nav_msgs/Odometry`，`camera_init → body` | 高频局部位姿；Mapping 和 Localization 的实时 odometry 输入 |
| `/cloud_registered` | `sensor_msgs/PointCloud2`，`camera_init` | 已配准到局部里程计坐标系的点云，主要用于显示 |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2`，`body` | 与 `/Odometry` 同时间戳；当前后端实际使用的点云 |
| TF `camera_init → body` | FAST-LIO 发布 | 实时局部 TF |

FAST-LIO 始终是 Mapping 和 Localization 的底层实时前端。后端不会替换、停止或向
ESIKF/ikd-tree 回写优化结果。FAST-LIO only 模式的 TF 只有 `camera_init → body`，不发布
`map → camera_init`。

## 3. Mapping：从开机到建图

### 3.1 启动顺序

机器人和 JT128 驱动已经启动、PC 接入机器人交换机后：

```bash
cd A2W_FastLio2
./scripts/check_lidar_topics.sh
./scripts/run_a2w_mapping.sh
```

不启动 RViz：

```bash
./scripts/run_a2w_mapping.sh rviz:=false
```

这是当前推荐的完整 Mapping 命令。它启动的 ROS2 进程是：

- `fast_lio/fastlio_mapping`：Hesai FAST-LIO 前端；
- `a2w_fastlio_mapping/mapping_ingress_node`：同步前端消息并选关键帧；
- `a2w_fastlio_mapping/mapping_backend_node`：回环、位姿图、优化输出、TF 和保存服务；
- `rviz2`：默认启动，可关闭。

Scan Context、LocalMapBuilder、Quatro、Nano-GICP、LoopValidator、GTSAM/iSAM2、
MapOdomManager 和 MapBundleService 是 `mapping_backend_node` 内部模块，不是可单独
`ros2 run` 的节点。

### 3.2 各模块何时运行

- FAST-LIO 全程实时运行，持续发布 `/Odometry` 和 `/cloud_registered_body`。
- `mapping_ingress_node` ExactTime 同步这两个 Topic。首帧必为关键帧；之后满足平移
  1.0 m、旋转 10°或间隔 2.0 s 任一条件时生成关键帧。
- 每个关键帧都会生成 Scan Context 描述子并最终写入描述子索引。没有 cooldown 且存在
  足够早的历史帧时才查询 Top-K；默认排除最近 30 帧。
- Quatro 和 Nano-GICP 不是每个传感器帧运行。它们只对通过 Scan Context 距离门槛的
  回环候选运行，并按 Top-K 顺序验证，接受一个后停止本窗口的后续候选。
- GTSAM/iSAM2 每个关键帧都会加入首帧 Prior 或相邻帧 Odometry factor，并执行增量
  `update()`；出现已验证回环时再额外加入 Loop factor，不是只有回环时才优化。
- 优化 odom/path/map preview 是关键帧级输出，不是 FAST-LIO 的传感器帧级输出。

### 3.3 Mapping 重要 Topic

| Topic | 发布者 | 内容 |
| --- | --- | --- |
| `/Odometry` | `fastlio_mapping` | 高频局部 odometry，`camera_init → body` |
| `/mapping/keyframe_odom` | `mapping_ingress_node` | 关键帧 odometry |
| `/mapping/keyframe_cloud` | `mapping_ingress_node` | 关键帧 body 点云 |
| `/mapping/registration_status` | `mapping_backend_node` | 候选、配准、验证、PGO、owner 审计 |
| `/mapping/optimized_odom` | `mapping_backend_node` | 最新关键帧的优化全局位姿，`map → body` |
| `/mapping/optimized_path` | `mapping_backend_node` | 全部优化关键帧路径，transient-local |
| `/mapping/optimized_map_preview` | `mapping_backend_node` | 降采样 RViz 地图，transient-local；不是完整地图 |
| `/a2w_fastlio/global_tf_owner` | 后端 | 全局 TF 单发布者心跳与冲突检测 |
| `/tf` | 前端和后端 | `camera_init → body` 与 `map → camera_init` |

当前没有独立的“Loop Candidate Topic”。候选和配准结果统一发布在
`/mapping/registration_status`。

## 4. 判断是否发生回环

持续查看全部回环审计：

```bash
ros2 topic echo /mapping/registration_status
```

只看已接受事件：

```bash
ros2 topic echo /mapping/registration_status --field accepted
```

查看消息定义：

```bash
ros2 interface show a2w_fastlio_msgs/msg/RegistrationStatus
```

Mapping 中字段解释：

| 字段 | 含义 |
| --- | --- |
| `keyframe_id` | 当前关键帧；接受回环时相当于 `from_id` |
| `candidate_id` | 历史候选；接受回环时相当于 `to_id` |
| `accepted` | 本候选或本阶段是否接受 |
| `stage` | `loop`、`pose_graph`、`ingress` 或 `ownership` |
| `reason` | `loop_accepted`、`no_loop_candidates`、`loop_policy_cooldown`、验证拒绝原因、`optimized_snapshot_updated` 等 |
| `fitness` | Nano-GICP 最终配准 fitness |
| `overlap` | 最终配准 overlap |
| `correspondence_count` | 最终对应点数量 |
| `elapsed_ms` | RegistrationPipeline 耗时 |

一次确认的回环至少应看到：

```text
stage: loop
accepted: true
reason: loop_accepted
keyframe_id: <current/from>
candidate_id: <history/to>
fitness: ...
overlap: ...
correspondence_count: ...
```

之后会看到：

```text
stage: pose_graph
accepted: true
reason: optimized_snapshot_updated
```

当前观测限制：

- Scan Context Top-K 没有独立消息；每个进入几何验证的候选体现为一条 `stage: loop` 记录。
- Quatro 的独立变换、内点数和耗时没有单独 Topic/字段，**当前未实现**独立 Quatro
  telemetry。最终 `fitness/overlap/correspondence_count` 是 Nano-GICP 后的统一结果。
- `RegistrationStatus` 没有 `factor_count` 字段；`loop_accepted` 证明 Loop factor 已被
  `addLoopConstraint()` 接受，`optimized_snapshot_updated` 证明本关键帧的 iSAM2 update 已完成，
  但不能仅靠 Topic 直接读取图中 factor 总数。
- 当前没有比 Topic 更完整的结构化 loop 日志文件；终端日志只适合辅助排错。

同时查看优化结果是否更新：

```bash
ros2 topic echo --once /mapping/optimized_odom
ros2 topic echo --once /mapping/optimized_path
ros2 topic echo --once /mapping/optimized_map_preview
ros2 run tf2_ros tf2_echo map camera_init
```

## 5. 保存和检查 Map Bundle

### 5.1 保存命令

Mapping 正在运行时，在另一个已按本文开头完成 ROS/CycloneDDS 初始化的终端执行：

```bash
ros2 service call /mapping/save_map_bundle \
  a2w_fastlio_msgs/srv/SaveMapBundle \
  "{output_path: factory_map}"
```

`factory_map` 是相对于 Mapping 启动时 `bundle_root` 的名字。通过
`./scripts/run_a2w_mapping.sh` 启动时，默认根目录是仓库的 `maps/`，因此输出为
`maps/factory_map/`。`output_path` 必须保持在该根目录内，不能使用根目录外的绝对路径或
`../` 逃逸路径。

保存时：

- 不要求停止机器人；
- 不要求停止 Mapping；
- 可以在 Mapping launch 仍运行时调用，但服务会等待保存 worker 写盘完成；当前
  `mapping_backend_node` 使用单线程 spin，等待期间该节点不会处理新的关键帧回调；
- 为得到路线终点处的最终地图，建议机器人安全静止，等待最后关键帧和
  `/mapping/registration_status` 的 `pose_graph` 更新完成，再调用服务，并保持静止直到
  Service 返回；FAST-LIO 和 ingress 是独立进程，期间仍可能产生消息，因此运动建图中保存
  可能造成队列积压或丢帧；
- 必须在停止 Mapping **之前**保存，按 `Ctrl+C` 后服务已经不存在；
- 同名目录再次保存会以经过校验的新 Bundle 原子替换旧 Bundle。

服务响应应检查 `success`、`message`、`bundle_uuid`、`keyframe_count` 和
`resolved_path`。只有 `success: true` 才算保存成功。

### 5.2 Bundle 内容

```text
maps/factory_map/
├── metadata.yaml
├── manifest.sha256
├── global_map.pcd
├── optimized_poses.csv
├── keyframes/
│   ├── index.csv
│   └── clouds/000000.pcd ...
├── descriptors/scan_context.bin
└── config/
    ├── mapping_effective.yaml
    ├── scan_context.yaml
    └── registration.yaml
```

`global_map.pcd` 是将关键帧 body 点云按 GTSAM 优化位姿重新拼接得到的完整优化地图；它
不是 FAST-LIO 的原始累计 PCD，也不是 `/mapping/optimized_map_preview`。关键帧原始局部
点云、FAST-LIO odom pose、优化 pose、Scan Context 描述子和有效参数快照都会保存。

完整性检查：

```bash
ros2 run a2w_fastlio_map map_bundle_inspect maps/factory_map
```

成功时输出 schema、UUID、关键帧数和硬件验证状态，并以 0 退出；manifest、缺失文件、
哈希、元数据或数据结构异常都会非零退出。

## 6. Localization：已有地图实时定位

完整启动：

```bash
./scripts/run_a2w_localization.sh maps/factory_map
```

不启动 RViz：

```bash
./scripts/run_a2w_localization.sh maps/factory_map rviz:=false
```

脚本先把路径解析为已存在的绝对目录并检查 `manifest.sha256`，然后启动：

- `fast_lio/fastlio_mapping`：持续高频局部 odometry；
- `a2w_fastlio_localization/localization_node`：只读加载并完整校验 Map Bundle，执行局部
  地图选择、Quatro、Nano-GICP、MatchValidator、状态监控和全局重定位；
- `rviz2`：默认启动。

Localization 是持续实时运行的，不是启动时定位一次后退出：

```text
JT128 + IMU
→ FAST-LIO 高频 camera_init → body
→ Localization 周期地图匹配更新 map → camera_init
→ 两次匹配之间用最新校正 × 高频 FAST-LIO odom 连续传播 map → body
→ /localization/pose、/localization/odom、/localization/path
```

FAST-LIO 频率由 JT128 点云/IMU频率、预处理、ESIKF/ikd-tree计算量和 PC 性能共同决定，
当前没有实机验证值。Localization 每个同步前端帧都传播位姿，但默认每 1000 ms 才尝试
一次地图匹配；参数 `match_interval_ms` 位于
`src/a2w_fastlio_localization/config/localization.yaml`。

| 输出 | 含义 |
| --- | --- |
| `/localization/pose` | 当前 `map → body` 的 `PoseStamped`，供一般全局 pose 消费者使用 |
| `/localization/odom` | 同一 `map → body` 的 `Odometry` 表达，便于需要 odometry 消息的组件使用；当前协方差未填充 |
| `/localization/path` | 已发布的全局轨迹，最多默认 10000 个 pose，主要用于 RViz/调试 |
| TF `map → camera_init` | 地图匹配维护的全局校正；与 FAST-LIO 的 `camera_init → body` 组成完整 TF 链 |

只有存在有效校正并且状态为 `LOCALIZED` 或 `DEGRADED` 时才发布上述定位输出和全局 TF；
`INITIALIZING`、`LOST`、`RELOCALIZING` 时会抑制旧全局结果。

## 7. Localization 冷启动如何获得初始位置

当前实际实现既不是完整的 A，也不是完整的 C：

- 人工 initial pose Topic/Service：**当前未实现**；
- 启动时不会直接执行全地图 Scan Context；
- 第一帧以 FAST-LIO 当前 `camera_init → body` pose 直接当作 `map → body` 预测，在其附近
  默认 30 m 范围内选择最多 20 个关键帧，执行 Quatro → Nano-GICP；
- 连续 2 次局部匹配成功后，从 `INITIALIZING` 进入 `LOCALIZED`。

这相当于选项 B，但前提是新启动的 FAST-LIO 原点与地图坐标中的起点足够接近，通常要求
机器人从建图起点附近重新启动。如果机器人在地图其他区域冷启动，附近关键帧选择可能
失败。

重要限制：当前状态机在 `INITIALIZING` 下记录 `initial_match_rejected`，但
INITIALIZING 不会仅因连续局部匹配失败自动进入 LOST，因此不会自动触发全地图重定位。
手动 initial pose 和冷启动全局定位均为**当前未实现**。这不影响“已经正常定位后进入
LOST”的自动 Global Relocalization，但在接 Nav2 前必须纳入实机验收和后续产品化工作。

## 8. 正常 Localization 与 Global Relocalization

正常 Localization 的真实链路是：

```text
最新 map → camera_init 校正（尚无校正时使用 FAST-LIO pose）
→ 与当前 FAST-LIO odom 合成全局预测 pose
→ 按半径选择附近 KeyFrames
→ 临时 Local Map
→ Quatro
→ Nano-GICP
→ MatchValidator
→ 跳变门限
→ 更新 map → camera_init
```

它默认低频运行，并利用先验全局预测缩小搜索范围。

已定位后进入 LOST 的 Global Relocalization 链路是：

```text
当前 body cloud
→ Scan Context 全地图 Top-K
→ 每个候选附近 KeyFrames 构造 Local Map
→ Quatro
→ Nano-GICP
→ MatchValidator
→ 候选综合得分与 best/second-best margin
→ 强结果或同一候选多帧一致确认
→ 原子恢复 map → camera_init
```

它不信任旧位置先验，并逐一审计 Top-K，不能用 Scan Context Top-1 直接确认位置。

## 9. 定位丢失与自动重定位

Global Relocalization 在已经进入 `LOST` 后自动触发。手动重定位 Service：当前未实现。

默认状态条件位于 `src/a2w_fastlio_localization/config/relocalization.yaml`：

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `monitor.initialization_successes_required` | 2 | 冷启动局部匹配确认数 |
| `monitor.degraded_failures_required` | 2 | 已定位后连续匹配失败达到该数进入 `DEGRADED` |
| `monitor.lost_failures_required` | 5 | 已定位/降级后连续匹配失败达到该数进入 `LOST` |
| `monitor.normal_recovery_successes_required` | 2 | `DEGRADED` 中连续成功后恢复 |
| `monitor.correction_stale_after_ms` | 3000 | 已定位/降级时校正年龄达到该值进入 `LOST` |
| `monitor.relocalization_timeout_ms` | 10000 | 单次重定位会话超时 |
| `global_relocalization.top_k` | 5 | 全地图 Scan Context 候选数 |
| `global_relocalization.evaluation_time_budget_ms` | 5000 | 单帧候选评估时间预算 |
| `global_relocalization.confirmation_count` | 3 | 非强结果所需一致帧数 |
| `global_relocalization.minimum_score_margin` | 0.05 | 最佳与次佳综合分差门限 |
| `global_relocalization.strong_maximum_fitness` | 0.08 | 强结果 fitness 上限 |
| `global_relocalization.strong_minimum_overlap` | 0.70 | 强结果 overlap 下限 |
| `global_relocalization.strong_minimum_correspondences` | 100 | 强结果对应点下限 |

状态流程：

- `INITIALIZING`：等待局部初始化成功；当前连续失败不会自动转 `LOST`。
- `LOCALIZED`：正常传播并周期匹配。连续 2 次匹配失败进入 `DEGRADED`；连续累计 5 次
  失败或校正超过 3 s 未刷新进入 `LOST`。
- `DEGRADED`：仍允许使用最近有效校正并发布全局结果；连续 2 次成功恢复
  `LOCALIZED`，失败累计达到 5 或校正过期则进入 `LOST`。
- `LOST`：立即停止旧全局输出，并自动创建重定位会话。
- `RELOCALIZING`：后台有界队列对新点云执行全局 Top-K。强结果可以单次确认，普通结果
  需同一候选且校正差在 1.0 m、0.25 rad 内连续 3 帧；确认后恢复 `LOCALIZED`。
- 重定位 10 s 超时会回到 `LOST`，后续输入会开始新会话。

`monitor.relocalization_successes_required` 当前仍是状态机公共参数；ROS 节点的 Stage 9
路径使用 `defer_confirmation`，实际恢复门槛由强结果或上述 `confirmation_count` 会话确认
完成。

## 10. 安全测试 LOST / Relocalization

当前没有运行时“注入错误 correction”“强制 LOST”“暂停 matcher”的测试 Service。停止
JT128/驱动或停止宇树服务不在允许范围内；仅仅让 Topic 消失也不会产生新的 monitor
证据，因此不能可靠触发 stale 转换。

最安全、可重复的方法是先使用仓库已有合成 launch test：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select a2w_fastlio_localization \
  --ctest-args -R test_global_relocalization_launch
colcon test-result --verbose
```

实机集中验收时，如机器人已经静止、未启动 Nav2，可使用一个临时参数副本把 stale 门槛
降到低于 1000 ms 匹配周期，使系统在成功初始化后自动进入 LOST；这不改仓库和机器人：

```bash
cp src/a2w_fastlio_localization/config/relocalization.yaml \
  /tmp/a2w_relocalization_force_lost.yaml
sed -i 's/correction_stale_after_ms: 3000/correction_stale_after_ms: 100/' \
  /tmp/a2w_relocalization_force_lost.yaml
./scripts/run_a2w_localization.sh maps/factory_map \
  relocalization_config:=/tmp/a2w_relocalization_force_lost.yaml
```

这会形成用于观察的重复 `LOCALIZED → LOST → RELOCALIZING` 条件，不是生产参数。测试时
保持机器人静止且不向底盘发送控制命令；结束后 `Ctrl+C`，正常运行时不要再传该临时文件。
能否恢复 `LOCALIZED` 仍取决于真实场景匹配，因此不得预先宣称通过。

## 11. 查看 Localization 与重定位状态

查看状态：

```bash
ros2 topic echo /localization/status
```

只看状态名或原因：

```bash
ros2 topic echo /localization/status --field state_label
ros2 topic echo /localization/status --field reason
```

查看普通定位和全局重定位的候选审计：

```bash
ros2 topic echo /localization/registration_status
```

`LocalizationStatus` 主要字段：

| 字段 | 含义 |
| --- | --- |
| `state` / `state_label` | 0..4 及 `INITIALIZING/LOCALIZED/DEGRADED/LOST/RELOCALIZING` |
| `reason` | 当前状态或最近转换原因 |
| `consecutive_successes` | 普通匹配连续成功计数 |
| `consecutive_failures` | 普通匹配连续失败计数 |
| `relocalization_successes` | 状态机重定位成功证据计数 |
| `correction_available` | 是否有可用全局校正 |
| `correction_age_ns` | 最近接受校正的年龄 |
| `candidate_id` | 最近候选关键帧 ID |
| `fitness/overlap/correspondence_count` | 最近配准质量 |
| `hardware_validation_pending` | 当前固定为 `true`，提醒尚未完成实机验收 |

`/localization/registration_status` 的 `stage: localization` 表示普通局部地图匹配，
`stage: global_relocalization` 表示 LOST 后的全地图候选审计。

## 12. 接入 Navigation 时是否持续运行

是。未来导航过程中 FAST-LIO 和 Localization 都必须始终在线：

```text
JT128 + IMU
        ↓
FAST_LIO_Hesai（持续）
        ↓ /Odometry + /cloud_registered_body
Localization（持续传播 + 周期纠偏）
        ↓ map → camera_init
TF: map → camera_init → body
        ↓
Nav2
        ↓
机器人控制
```

Localization 不是启动时定位一次以后停止。当前代码已经实现高频 odometry 传播与低频
地图匹配纠偏，但尚未提供 Nav2 launch、`base_link` 静态变换、Nav2 参数或底盘控制集成。
真实 frame、频率、延迟和控制安全仍待实机验证。

## 13. 导航中发生定位丢失时的接口

当前 `/localization/status` 已足以让未来的安全监督节点按 `state_label` 做基础门控：

```text
LOCALIZED     → 允许 Nav2 正常运行
DEGRADED      → 可选择降速
LOST          → 取消/暂停 Goal，并请求机器人停止
RELOCALIZING  → 保持停止
LOCALIZED     → 重新规划后继续
```

但本仓库当前没有实现 Nav2 lifecycle 管理、Goal 取消、速度限制或 Robot Stop。建议正式
Nav2 集成前补充或由安全监督节点维护：

- 明确的 `safe_to_navigate`/severity 输出；
- 状态序号或 session ID，避免漏掉快速转换；
- 定位协方差/可信度；
- 最近有效全局校正的时间戳（当前只能由 status stamp 与 `correction_age_ns` 推导）；
- Nav2 暂停、停止确认和恢复握手。

本轮不实现这些接口。

## 14. Mapping 与 Localization 的标准切换

第一次建图：

```text
1. ./scripts/run_a2w_mapping.sh
2. 完成建图路线
3. 机器人安全静止并等待最后一次 pose_graph 更新
4. 调用 /mapping/save_map_bundle，确认 success: true
5. map_bundle_inspect 检查通过
6. Ctrl+C 停止 Mapping
```

以后定位：

```text
1. 确认 Mapping 已停止
2. ./scripts/run_a2w_localization.sh maps/factory_map
3. 观察 /localization/status
4. 等到 LOCALIZED
5. 以后再启动 Nav2（当前尚未集成）
```

Mapping 与 Localization 绝对不能同时启动。Mapping 中 `map → camera_init` 由
`mapping_backend_node` 发布；Localization 中由 `localization_node` 发布。二者通过
`/a2w_fastlio/global_tf_owner` 检测冲突，但这个保护不能替代正确的操作顺序。

## 15. 集中实机验收脚本

`run_jt128_full_validation.sh` 只观察当前 ROS graph 并收集证据，不负责启动系统，也不会
修改机器人。验收时至少开两个终端：一个运行被测模式，一个运行验收脚本。

### 15.1 Mapping 验收

先启动 Mapping、完成包含真实回环的路线并保存 Bundle，然后在 Mapping 仍运行时执行：

```bash
./scripts/run_jt128_full_validation.sh \
  --mode mapping \
  --duration 30 \
  --map-bundle maps/factory_map \
  --output-dir log/jt128_validation/mapping
```

窗口内没有关键帧、接受回环或相应审计证据时，相关检查会失败；这不是脚本错误。

### 15.2 Localization / Relocalization 验收

停止 Mapping，启动 Localization。要让脚本通过 global relocalization 项，采集窗口内必须
真实观察到 `LOST → RELOCALIZING → LOCALIZED`；可按第 10 节使用临时 stale 配置：

```bash
./scripts/run_jt128_full_validation.sh \
  --mode localization \
  --duration 30 \
  --map-bundle maps/factory_map \
  --output-dir log/jt128_validation/localization
```

### 15.3 Mapping + Localization 全部验收

不要在同一时刻运行两个模式，也不要用单次在线 `--mode all` 强行验收。当前 `all` 会同时
要求 Mapping 和 Localization Topic/状态出现在一个采集窗口，而正确架构禁止二者共存；
它只能安全用于列出检查项：

```bash
./scripts/run_jt128_full_validation.sh --dry-run --mode all
```

真正的完整验收是按 15.1 和 15.2 顺序运行两次、使用两个输出目录，再联合审阅两份报告。
“跨两个互斥会话自动合并报告”的命令：**当前未实现**。

每次输出：

- `validation_summary.yaml`：机器可读的 mode、总状态、硬件验证状态和每项 pass/fail；
- `validation_report.md`：人工阅读表格，包含每项状态和简要原因；
- `raw/`：每项检查的原始 `ros2`、TF、频率、资源和延迟证据，失败时必须优先查看。

只有所有适用检查均通过时报告才写 `hardware_validation_status: validated`。dry-run、缺少
Topic、缺少 Bundle、没有发生回环/重定位或任何检查失败都会保持 `pending` 并通常非零退出。

## 16. 现场最简命令清单

### A. 只运行 FAST-LIO

```bash
./scripts/run_fastlio_jt128_pc.sh
```

### B. 建图

```bash
./scripts/run_a2w_mapping.sh
```

### C. 查看回环

```bash
ros2 topic echo /mapping/registration_status
```

### D. 保存地图

```bash
ros2 service call /mapping/save_map_bundle \
  a2w_fastlio_msgs/srv/SaveMapBundle "{output_path: factory_map}"
```

### E. 检查地图

```bash
ros2 run a2w_fastlio_map map_bundle_inspect maps/factory_map
```

### F. 使用已有地图定位

```bash
./scripts/run_a2w_localization.sh maps/factory_map
```

### G. 查看定位状态

```bash
ros2 topic echo /localization/status
```

### H. 查看重定位过程

```bash
ros2 topic echo /localization/registration_status
```

关注 `stage: global_relocalization`；手动触发 Service 当前未实现。

### I. Mapping 实机验收

```bash
./scripts/run_jt128_full_validation.sh \
  --mode mapping --duration 30 \
  --map-bundle maps/factory_map \
  --output-dir log/jt128_validation/mapping
```

### J. Localization / Relocalization 实机验收

```bash
./scripts/run_jt128_full_validation.sh \
  --mode localization --duration 30 \
  --map-bundle maps/factory_map \
  --output-dir log/jt128_validation/localization
```

## 17. 完整运行数据流和责任边界

### 17.1 Mapping

```text
JT128 /lidar_points + /lidar_imu
  → [fast_lio/fastlio_mapping]
  → /Odometry + /cloud_registered_body
  → [a2w_fastlio_mapping/mapping_ingress_node]
  → /mapping/keyframe_odom + /mapping/keyframe_cloud
  → [a2w_fastlio_mapping/mapping_backend_node]
       → [a2w_fastlio_common: Scan Context]
       → Top-K candidate
       → [a2w_fastlio_common: LocalMapBuilder + Quatro + Nano-GICP + MatchValidator]
       → [a2w_fastlio_mapping: LoopPipeline + GTSAM/iSAM2]
       → /mapping/registration_status
       → /mapping/optimized_odom + /mapping/optimized_path
       → /mapping/optimized_map_preview
       → TF map → camera_init
       → [a2w_fastlio_map: Map Bundle Writer]
       → maps/<name>/global_map.pcd + Bundle data
```

| 段 | package / node | 输入 | 输出 | YAML |
| --- | --- | --- | --- | --- |
| FAST-LIO | `fast_lio/fastlio_mapping` | `/lidar_points`, `/lidar_imu` | `/Odometry`, `/cloud_registered_body`, `camera_init → body` | `src/a2w_fastlio2_bringup/config/jt128.yaml` |
| KeyFrame | `a2w_fastlio_mapping/mapping_ingress_node` | 前端 odom/cloud | `/mapping/keyframe_odom`, `/mapping/keyframe_cloud` | `src/a2w_fastlio_mapping/config/mapping.yaml` |
| Loop/PGO/Map | `a2w_fastlio_mapping/mapping_backend_node` | 关键帧 Topic | registration status、优化输出、`map → camera_init` | `mapping_topics.yaml`, `scan_context.yaml`, `registration.yaml`, `loop_validation.yaml`, `pose_graph.yaml` |
| Map Bundle | `mapping_backend_node` 内服务 + `a2w_fastlio_map` 库 | 后端一致快照、`/mapping/save_map_bundle` | `maps/<name>/` | `src/a2w_fastlio_map/config/map.yaml` |

启动命令：`./scripts/run_a2w_mapping.sh`。

### 17.2 Localization

```text
JT128
→ [fast_lio/fastlio_mapping]
→ /Odometry + /cloud_registered_body
→ [a2w_fastlio_localization/localization_node]
   + 只读 Map Bundle
   → LocalMapSelector
   → Quatro → Nano-GICP → MatchValidator（默认每 1 s）
   → map → camera_init correction
   → 每个前端帧传播 map → body
   → /localization/pose + /localization/odom + /localization/path
   → /localization/status + /localization/registration_status
```

| 段 | package / node | 输入 | 输出 | YAML |
| --- | --- | --- | --- | --- |
| FAST-LIO | `fast_lio/fastlio_mapping` | JT128 Topic | 前端 odom/body cloud、局部 TF | `src/a2w_fastlio2_bringup/config/jt128.yaml` |
| Bundle load | `a2w_fastlio_localization/localization_node` + `a2w_fastlio_map` reader | `maps/<name>/` | 只读 snapshot | Bundle 自带 metadata/config |
| 定位 | `a2w_fastlio_localization/localization_node` | odom/body cloud + snapshot | 全局 pose/odom/path、状态、`map → camera_init` | `localization.yaml`, `localization_topics.yaml`, `map_matching.yaml`, `relocalization.yaml` |

启动命令：`./scripts/run_a2w_localization.sh maps/factory_map`。

### 17.3 Relocalization

```text
已定位后 LOST
→ [localization_node: LocalizationMonitor]
→ RELOCALIZING
→ [a2w_fastlio_common: Scan Context 全地图 Top-K]
→ [GlobalRelocalizer: candidate Local Map]
→ [a2w_fastlio_common: Quatro → Nano-GICP → MatchValidator]
→ [RelocalizationSession: ambiguity + strong/multiframe confirmation]
→ 恢复 map → camera_init
→ LOCALIZED + 恢复全局 pose/odom/path
```

| 段 | package / node | 输入 | 输出 | YAML |
| --- | --- | --- | --- | --- |
| 状态检测 | `a2w_fastlio_localization/localization_node` | 普通匹配证据、校正年龄 | `/localization/status` | `relocalization.yaml` 的 `monitor.*` |
| 全局候选/配准 | 同一 `localization_node`，共享 `a2w_fastlio_common` 接口 | 当前 body cloud、Bundle descriptors/keyframes | `/localization/registration_status`，`stage=global_relocalization` | `relocalization.yaml` + `map_matching.yaml` |
| 恢复 | 同一 `localization_node` | 已确认 correction | `LOCALIZED`、TF 与定位输出恢复 | `relocalization.yaml` |

Relocalization 没有独立启动命令；它随 Localization 启动，并只在正常定位之后进入
`LOST` 时自动运行。

---

最终状态：`Offline implementation and verification complete; JT128 hardware validation pending.`

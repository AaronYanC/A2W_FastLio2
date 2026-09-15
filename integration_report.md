# A2W Hesai JT128 SLAM / Localization 集成报告（Stage 0）

生成日期：2026-09-15
范围：只读检查与集成设计；本阶段不修改算法代码、不连接机器人、不执行实机测试。

## 1. 结论

当前工程可以作为后续 SLAM、Localization 和 Global Relocalization 的唯一前端基线，
不需要替换或重新适配 `FAST_LIO_Hesai`。后端可从现有 `/Odometry` 与
`/cloud_registered` 接入，并使用两者相同的 LiDAR 扫描结束时间戳生成关键帧。

推荐保持松耦合：

```text
Hesai JT128 Driver（机器人端，不改）
        ↓ /lidar_points + /lidar_imu
FAST_LIO_Hesai（现有 ROS2 前端，不改算法）
        ↓ /Odometry + /cloud_registered
共享算法层（Scan Context / Quatro / Nano-GICP）
        ├── Mapping Backend（GTSAM/iSAM2）
        └── Localization / Relocalization
```

有两个实施前置条件：

1. 当前前端的真实 frame 是 `camera_init → body`，不是需求文档中的
   `odom → base_link`。在没有机器人 TF 快照和传感器消息 header 前，不能猜测
   `body`、`base_link`、LiDAR、IMU 的外参关系。Stage 1 前必须确定 frame policy。
2. 两个参考仓库不是“可无限制复制”的宽松许可证工程。主仓库和 Scan Context
   声明 CC BY-NC-SA 4.0（禁止商业用途并要求同方式共享），Quatro 的许可证文件为
   GPL-3.0、README 又声明 CC BY-NC-SA，存在许可证口径冲突。是否直接移植源码必须
   先确认本项目用途和发布方式。

## 2. 检查基线与证据

### 2.1 当前仓库

| 项目 | 检查结果 |
| --- | --- |
| OS | Ubuntu 22.04，amd64/x86_64 |
| ROS | ROS 2 Humble |
| RMW | 项目运行脚本明确导出 `rmw_cyclonedds_cpp`；本次非运行 shell 中未预设该变量 |
| 构建 | `colcon build --symlink-install` + `ament_cmake`，C++17 |
| colcon-core | 0.21.0 |
| CMake | 3.22.1 |
| GCC/G++ | 11.4.0 |
| 当前分支 | `main`，跟踪 `origin/main` |
| HEAD | `bf6848e` (`feat: make A2W JT128 FAST-LIO2 workspace portable`) |
| Git 状态 | 检查时 clean，无未提交修改 |
| FAST_LIO_Hesai | Git submodule，ROS2 分支固定在 `16e97cdcc4260b9ba6241518a19ad8384224ba48` |
| ikd-Tree | 递归 submodule，固定在 `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4` |
| 地图文件 | 本机 `maps/` 下有运行产物；`*.pcd` 已由 `.gitignore` 排除，不纳入本次提交 |

当前工程结构：

```text
.
├── config/                         # CycloneDDS 模板
├── maps/                           # 运行时地图，PCD 不进 Git
├── scripts/                        # bootstrap/build/run/topic check
├── src/
│   ├── FAST_LIO_Hesai/             # 现有 Hesai ROS2 前端 submodule
│   └── a2w_fastlio2_bringup/       # JT128 参数和 launch
└── tests/                          # 可迁移性与配置测试
```

### 2.2 JT128 当前配置

配置来自 `src/a2w_fastlio2_bringup/config/jt128.yaml`：

| 参数 | 当前值 |
| --- | --- |
| `preprocess.lidar_type` | `2`（JT128 分支） |
| `preprocess.scan_line` | `128` |
| `preprocess.timestamp_unit` | `0`（SEC；转换到内部毫秒） |
| `preprocess.blind` | `0.5 m` |
| `point_filter_num` | `6` |
| `filter_size_surf` | `0.5 m` |
| `filter_size_map` | `0.3 m` |
| `mapping.det_range` | `50.0 m` |
| `common.time_sync_en` | `false` |
| `common.time_offset_lidar_to_imu` | `0.0 s` |
| `common.imu_gyr_unit` | `auto` |
| `mapping.extrinsic_est_en` | `true` |
| `mapping.extrinsic_T` | `[0, 0, 0]` |
| `mapping.extrinsic_R` | 单位矩阵 |

这些值是当前稳定链路基线。本集成不修改 JT128 解析、IMU 解析、ESIKF、ikd-tree
或上述外参配置。需要注意：`extrinsic_est_en: true` 表示前端当前允许在线估计外参；
本报告只记录事实，不对其有效性作额外推断。

## 3. 当前 FAST_LIO_Hesai 接口

### 3.1 Topic 表（代码确认）

| 方向 | Topic | ROS 2 类型 | QoS/队列 | frame_id | 时间戳/频率 |
| --- | --- | --- | --- | --- | --- |
| 输入 | `/lidar_points` | `sensor_msgs/msg/PointCloud2` | `SensorDataQoS` | 未联机，不能从本仓库确认驱动 header | 驱动 header stamp；实测频率待联机 |
| 输入 | `/lidar_imu` | `sensor_msgs/msg/Imu` | `SensorDataQoS` | 未联机，不能从本仓库确认驱动 header | 驱动 header stamp；实测频率待联机 |
| 输出 | `/Odometry` | `nav_msgs/msg/Odometry` | depth 20 | parent=`camera_init`，child=`body` | `lidar_end_time`；每个成功同步并处理的扫描一次 |
| 输出 | `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | depth 20 | `camera_init` | 与 `/Odometry` 相同的 `lidar_end_time`；启用 scan publish 时每个处理扫描一次 |
| 输出 | `/cloud_registered_body` | `sensor_msgs/msg/PointCloud2` | depth 20 | `body` | `lidar_end_time`；当前配置已启用 |
| 输出 | `/cloud_effected` | `sensor_msgs/msg/PointCloud2` | depth 20 | `camera_init` | `lidar_end_time`；当前默认不开启 |
| 输出 | `/Laser_map` | `sensor_msgs/msg/PointCloud2` | depth 20 | `camera_init` | 1 Hz timer；当前配置未启用 `publish.map_en` |
| 输出 | `/path` | `nav_msgs/msg/Path` | depth 20 | `camera_init` | 每 10 个已处理扫描追加并发布一次 |
| 服务 | `/map_save` | `std_srvs/srv/Trigger` | — | — | 仅 `pcd_save_en=true` 且缓存非空时成功 |

前端内部点类型是 `pcl::PointXYZINormal`。ROS 1 参考后端读取成
`pcl::PointXYZI`，多余字段可由 PCL 转换忽略；ROS2 后端仍应写字段兼容测试，不能只
假设所有 `PointCloud2` 布局完全相同。

### 3.2 时间语义

- JT128 PointCloud2 header stamp 被当作扫描开始时间 `lidar_beg_time`。
- `preprocess.timestamp_unit=SEC` 时，每点 `timestamp` 差值乘 `1e3`，存到 PCL
  `curvature` 字段，单位为毫秒。
- 前端用最后一点的相对时间估算 `lidar_end_time`，并等待 IMU 覆盖到该时刻。
- `/Odometry`、`/cloud_registered`、`/cloud_registered_body` 和动态 TF 均使用
  同一个 `lidar_end_time`。
- 节点处理 timer 为 100 Hz，但它只是轮询同步缓存，不等于输出 100 Hz。真实输出
  频率取决于 JT128 扫描频率和同步成功率；本次未连接机器人，因此不填写猜测值。

由此，Stage 1 可以优先按完全相同 stamp 配对 odom/cloud，并对超时、乱序、重复
stamp 进行统计；不建议照搬 ROS1 上游的无约束 ApproximateTime 行为。

### 3.3 当前 TF

代码确认 FAST-LIO 每个已处理扫描广播：

```text
camera_init → body
```

变换与 `/Odometry.pose.pose` 相同，时间戳为 `lidar_end_time`。当前仓库没有代码广播：

```text
map → odom
odom → base_link
base_link → lidar
base_link → imu
```

机器人端可能有其他 TF publisher，但不在当前仓库内，本次又没有连接机器人，故不能
把它写成已确认事实。`preprocess.cpp` 中局部转换消息曾写入 `livox` frame，也不能
当作 JT128 输入消息的真实 frame_id 证据。

### 3.4 当前 PCD 与 trajectory 保存

当前 PCD 保存有两种入口，共用前端累计点云缓存：

1. `./scripts/run_fastlio_jt128_mapping.sh` 把 `save_map:=true` 和仓库根目录解析出的
   `maps/jt128_map.pcd` 传给 launch；正常 `Ctrl+C` 时保存二进制 PCD。
2. 建图过程中调用 `/map_save` Trigger 服务立即刷新到 `map_file_path`。

当前 `pcd_save.interval=-1`，不会按扫描数量拆分保存；`leaf_size=0.1 m`，保存前会
体素降采样。YAML 默认 `pcd_save_en=false`，mapping 脚本通过 launch 覆盖为 true。

前端 trajectory 保存由 `trajectory_save.tum_en` 控制，当前为 false。启用后写入
FAST_LIO 源码目录编译期 `ROOT_DIR` 下的 `Log/traj_tum.txt`，每行格式为：

```text
timestamp x y z qx qy qz qw
```

该路径不适合未来 Map Bundle。后续优化轨迹应由 Mapping Backend 保存；无需为此
修改或启用前端 trajectory 输出。

## 4. 两个参考仓库检查

检查对象及固定版本：

- [FAST-LIO-SAM-SC-QN](https://github.com/engcang/FAST-LIO-SAM-SC-QN/tree/2ab179a3ec7254783553dd1b263a6cd6e819b8dc)：`2ab179a3ec7254783553dd1b263a6cd6e819b8dc`
- [FAST-LIO-Localization-SC-QN](https://github.com/engcang/FAST-LIO-Localization-SC-QN/tree/fb6d1cc309e6169d1791d853a0f27b8cc56ad643)：`fb6d1cc309e6169d1791d853a0f27b8cc56ad643`

两者均为 ROS1/catkin，不是 ROS2 包。它们使用 `ros::NodeHandle`、ROS1
`message_filters`、`tf`、XML launch 和 ROS1 bag，因此只能移植算法和流程，不能
直接放进 Humble workspace。

### 4.1 Mapping 上游实际行为

- 输入固定为 `/Odometry` + `/cloud_registered`，ApproximateTime queue 10。
- 关键帧只使用平移距离阈值；没有旋转阈值和最大时间间隔。
- Scan Context 内部 KD-tree 查询 3 个候选，但对外 API 和 Mapping Backend 最终只
  返回 Top-1。
- 候选又按当前优化位姿的欧氏距离裁剪，不适合作为真正全局重定位。
- Quatro 做粗配准，Nano-GICP 做精配准。
- Loop 接受条件只有 Nano-GICP `hasConverged()` + fitness threshold，不满足本项目
  对重复厂区的验证要求。
- GTSAM 图包含 Prior、相邻关键帧 Between、Loop Between，使用 iSAM2 增量更新。
- 它直接广播 `map → robot`，不是本项目要求的 `map → odom`。
- 地图输出可保存全局 PCD、逐关键帧 PCD、KITTI/TUM pose 和 ROS1 bag；ROS1 bag
  以 `/keyframe_pcd`、`/keyframe_pose` 供定位仓库加载。

### 4.2 Localization 上游实际行为

- 输入同样固定为 `/Odometry` + `/cloud_registered`。
- 地图必须是 ROS1 bag，且示例配置含开发者本机绝对路径。
- 地图加载时重新构建 Scan Context 数据库。
- 正常匹配仍只使用 Scan Context Top-1，并用当前 corrected pose 的距离门限裁剪；
  这不是 LOST 后的全地图重定位。
- 有 Quatro 时 target 是一个候选关键帧；无 Quatro 时 target 才是附近关键帧拼成的
  local map。这个限制说明 source/target 尺度必须在新设计中显式管理。
- 匹配失败后继续沿用 `last_corrected_TF`；没有 INITIALIZING/LOCALIZED/DEGRADED/
  LOST/RELOCALIZING 状态机。
- 接受条件同样只有 Nano-GICP convergence + fitness。
- 它直接广播 `map → robot`，不能原样作为本项目 TF owner。

### 4.3 可参考与禁止引入的边界

| 内容 | 决策 | 说明 |
| --- | --- | --- |
| 上游 `third_party/FAST_LIO` | 禁止引入 | 当前 Hesai ROS2 前端是唯一前端 |
| 上游 Livox driver | 禁止引入 | JT128 Driver 机器人端保持不动 |
| ROS1 node/launch/bag glue | 不复制 | 重新实现为 ROS2/ament 接口 |
| Keyframe/PGO 数据流 | 参考 | 加入旋转/时间阈值、线程隔离和参数化 |
| Scan Context | 需改造 | 共享算法模块，API 必须返回 Top-K 和 score/yaw |
| Quatro | 需独立封装 | 共享一份，算法层不依赖 ROS；先解决许可证与 TEASER++ |
| Nano-GICP | 可独立封装 | MIT；共享一份，算法层不依赖 ROS |
| GTSAM/iSAM2 | 系统依赖 | Mapping only；不回写 ESIKF |
| ROS1 bag map | 不作为主格式 | 统一 Map Bundle；legacy loader 后置 |

## 5. Mapping 对接设计

Mapping Backend 订阅现有 `/Odometry` 和 `/cloud_registered`。两条消息同 stamp 后
生成 `FrontendFrame`，不修改 FAST_LIO 源码。

```text
FrontendFrame
  → KeyFrameManager
  → ScanContextManager::queryTopK()
  → LocalMapBuilder
  → QuatroRegistration
  → NanoGicpRegistration
  → LoopValidator
  → PoseGraphOptimizer (GTSAM/iSAM2)
  → MapOdomManager
  → MapManager / MapBundleWriter
```

关键设计：

- Keyframe 条件为平移、旋转、最大时间间隔的 OR，初值 1.0 m / 10 deg / 2 s，全部
  YAML 化。
- 保存的 keyframe cloud 必须处于明确的 keyframe/tracking local frame。由于
  `/cloud_registered` 已在 `camera_init`，应使用同 stamp 的 odom pose 逆变换回局部
  frame，并以单元测试验证，与上游 `PosePcd` 的核心处理一致。
- Scan Context 只给候选，不接受 loop；返回 Top-K 的 `id/score/yaw_hint`。
- LocalMapBuilder 动态拼接候选邻域关键帧，不建立持久化 Submap Manager。
- Quatro source/target 使用相近尺度的 current local cloud 与 candidate local cloud；
  禁止 single scan 对整张 global map。
- Nano-GICP 接收 Quatro initial transform，再输出精配准 transform 和统计量。
- V1 LoopValidator 真正启用 convergence、fitness、时间/关键帧分离；接口同时预留
  overlap、correspondence、跳变量和多帧一致性。
- 图优化只修正 keyframe global pose。高频 FAST_LIO odometry、ESIKF 与 ikd-tree
  均不回写。
- 重计算放独立 callback group/worker；接收前端数据的 callback 只做配对、轻量转换
  和入队，使用有界队列防止内存无限增长。

## 6. Localization 与 Relocalization 对接设计

Localization 使用同一个 `FrontendFrame` 接口和同一套 shared algorithms：

```text
MapBundleReader
  → KeyframeDatabase + DescriptorDatabase

/Odometry + /cloud_registered
  → KeyFrameManager
  → LocalizationManager
      ├── 当前 T_map_odom → 高频 global pose/odom/path
      └── 低频 map_match_hz
            → LocalMapSelector
            → Quatro → Nano-GICP → MatchValidator
            → 更新 T_map_odom
```

状态机：

```text
INITIALIZING → LOCALIZED → DEGRADED → LOST → RELOCALIZING → LOCALIZED
```

- LOCALIZED：使用当前位置附近的 keyframe 构建 local target，低频纠正。
- DEGRADED：短期继续使用最近一次可靠 `T_map_odom`，同时提高告警/匹配策略。
- LOST：不再把旧 corrected pose 当作全局检索门限。
- RELOCALIZING：当前 local cloud 对 Map Bundle 的 Scan Context DB 做 Top-K 全图检索，
  各候选依次 Quatro + Nano-GICP + Validator，选择最高置信结果。
- 恢复需要可配置的一次强验证或多帧一致性确认，不能 Scan Context Top-1 直接接受。
- Localization 模式不添加 Pose Graph 节点、不修改保存地图。

`MapMatchResult` 建议至少包含：

```text
success, candidate_id, transform, scan_context_score, yaw_hint,
coarse_success, fine_success, fitness, overlap, correspondence_count,
translation_delta, rotation_delta, elapsed_ms, rejection_reason
```

## 7. Topic / TF 对接表

### 7.1 新后端 Topic（建议默认值，均应可参数化）

| 模式 | 方向 | Topic | 类型 | 说明 |
| --- | --- | --- | --- | --- |
| 两者 | 输入 | `/Odometry` | `nav_msgs/msg/Odometry` | FAST_LIO local pose |
| 两者 | 输入 | `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | FAST_LIO world/local-odom frame cloud |
| Mapping | 输出 | `/mapping/optimized_odom` | `nav_msgs/msg/Odometry` | 优化后的关键帧/实时全局 pose |
| Mapping | 输出 | `/mapping/optimized_path` | `nav_msgs/msg/Path` | 优化轨迹 |
| Mapping | 输出 | `/mapping/optimized_map` | `sensor_msgs/msg/PointCloud2` | 低频或按需发布 |
| Mapping | 输出 | `/mapping/loop_markers` | `visualization_msgs/msg/MarkerArray` | candidate/accepted/rejected loop |
| Mapping | 服务 | `/mapping/save_map` | 建议自定义 service 或 Trigger + 参数 | 原子写 Map Bundle |
| Localization | 输出 | `/localization/pose` | `geometry_msgs/msg/PoseStamped` | map frame 实时位姿 |
| Localization | 输出 | `/localization/odom` | `nav_msgs/msg/Odometry` | map frame 全局里程计 |
| Localization | 输出 | `/localization/path` | `nav_msgs/msg/Path` | 全局路径 |
| Localization | 输出 | `/localization/status` | 自定义 `LocalizationStatus` | 状态、质量和失败计数 |
| Localization | 输出 | `/localization/current_cloud` | `sensor_msgs/msg/PointCloud2` | 当前对齐云 |
| Localization | 输出 | `/localization/local_map` | `sensor_msgs/msg/PointCloud2` | 当前候选局部地图 |
| 两者 | Debug | `/registration/coarse_aligned` | `sensor_msgs/msg/PointCloud2` | Quatro 结果 |
| 两者 | Debug | `/registration/fine_aligned` | `sensor_msgs/msg/PointCloud2` | Nano-GICP 结果 |
| 两者 | Debug | `/registration/match_markers` | `visualization_msgs/msg/MarkerArray` | 匹配与验证信息 |

### 7.2 TF owner

目标约束：任一运行模式中只能有一个 `map → local_odom_frame` owner。

| 模式 | TF | Owner | 备注 |
| --- | --- | --- | --- |
| Mapping | local odom → tracking body | FAST_LIO_Hesai | 当前实际名称 `camera_init → body` |
| Mapping | map → local odom | Mapping `MapOdomManager` | 由最新优化 pose 与同步 FAST_LIO pose 计算 |
| Localization | local odom → tracking body | FAST_LIO_Hesai | 当前实际名称 `camera_init → body` |
| Localization | map → local odom | `LocalizationManager` | Mapping Backend 不得同时运行/广播 |

计算约定：

```text
T_map_local_odom = T_map_tracking_optimized × inverse(T_local_odom_tracking_fastlio)
```

### 7.3 Frame policy 决策门

当前零侵入可立即使用的真实链是：

```text
map → camera_init → body
```

最终要求的 `map → odom → base_link` 不能靠重命名文档完成。Stage 1 前需联机采集：

```text
ros2 topic echo --once /Odometry
ros2 topic echo --once /cloud_registered/header
ros2 topic echo --once /lidar_points/header
ros2 topic echo --once /lidar_imu/header
ros2 run tf2_tools view_frames
```

之后从以下方案中选定一项：

1. **保持原 frame（首版最稳）**：后端参数设为
   `local_odom_frame=camera_init`、`tracking_frame=body`，发布
   `map→camera_init`。不触碰前端，但暂不宣称满足 Nav2 标准命名。
2. **增加独立 TF adapter（推荐的最终方案）**：从 `/Odometry` 广播参数化的
   `odom→base_link`，并在确认机器人静态 TF 后连接传感器 frame。原
   `camera_init→body` 保留为兼容树；必须验证无多 parent、无 TF loop。
3. **以后给前端 frame 参数化**：只改 frame 字符串，不改算法，但属于对现有前端的
   最小侵入，需单独批准后才能做。

在实机 frame 关系未确认前，报告不选择 2/3，也不假设 `body == base_link`。

## 8. Map Bundle V1

目录建议：

```text
maps/<map_id>/
├── metadata.yaml
├── map_manifest.yaml
├── global_map.pcd
├── optimized_trajectory.txt
├── keyframes/
│   ├── 000000.pcd
│   └── ...
├── poses/
│   └── optimized_poses.txt
└── descriptors/
    ├── scan_context.bin
    └── scan_context_index.yaml
```

数据约定：

- `metadata.yaml`：`format_version`、`map_id`、创建时间、单位、frame 名称、传感器
  型号、前后端 commit、配置摘要/hash、点类型、关键帧数量、地图 bounds。
- `map_manifest.yaml`：每个文件的相对路径、格式、记录数、字节数、SHA-256；禁止
  绝对路径。
- `keyframes/*.pcd`：binary compressed PCD，`x/y/z/intensity`，在明确的 keyframe
  local/tracking frame 中。
- `optimized_poses.txt`：一行一个 keyframe，建议
  `id timestamp tx ty tz qx qy qz qw`，pose 是 `T_map_keyframe`。
- `optimized_trajectory.txt`：同样使用 TUM 风格，作为评估/可视化输出。
- `scan_context.bin`：带 magic、schema version、endianness、rows、cols、记录数；
  index 文件保存 keyframe id、offset、ring key/yaw 配置。不要依赖 C++ 对象内存布局。
- `global_map.pcd`：由 keyframe local clouds + optimized poses 重建并降采样，不直接
  使用前端未优化累计 PCD 作为最终优化地图。

写入应先输出到同目录临时文件，全部成功后原子 rename；加载时检查版本、数量、
hash、frame 和 descriptor 参数一致性。Mapping writer 与 Localization reader 必须
共用同一个 `a2w_fastlio_map` 实现，不能各写一套格式。

## 9. 第三方依赖

### 9.1 当前机器已安装

| 依赖 | 当前版本/状态 | 计划 |
| --- | --- | --- |
| Eigen | 3.4.0 | 复用系统版本，不替换 |
| PCL | 1.12.1 | 复用系统版本，不替换 |
| GTSAM | 4.1.1 | 已有系统 `libgtsam-dev`，Stage 4 做最小编译验证 |
| TBB | 2021.5.0 | 已安装；Quatro 是否启用 TBB 由构建选项控制 |
| Boost | 1.74 | 系统已有 |
| OpenMP | 编译器 `_OPENMP=201511`，另有 libomp 14 | 使用 CMake target，不改系统工具链 |
| TEASER++ | 未发现 CMake package 或动态库 | Quatro 前的唯一明确缺失依赖 |
| Scan Context | 当前 workspace 未安装 | 待许可证决策后作为共享算法模块引入/重写 |
| Quatro | 当前 workspace 未安装 | 待许可证决策；依赖 TEASER++ |
| Nano-GICP | 当前 workspace 未安装 | MIT，可固定上游 commit 后封装为 ament library |

### 9.2 上游版本参考

Mapping 仓库记录的 submodule 版本：

| 组件 | commit | 许可证检查 |
| --- | --- | --- |
| Quatro | `d27109bd6a1798e9cf2e0c2a4daac6af16e7bc23` | `License` 为 GPL-3.0；README 又称 CC BY-NC-SA，需澄清 |
| Nano-GICP | `b21e79edcceb7c6e86ec0dfec90f451b796b683a` | MIT |
| scancontext_tro | `c8ef5b496a159cdfd7fa4761121178f25cd0a6bb` | README 声明 CC BY-NC-SA 4.0 |

主参考仓库的 [LICENSE](https://github.com/engcang/FAST-LIO-SAM-SC-QN/blob/master/LICENSE)
和定位仓库的 [LICENSE](https://github.com/engcang/FAST-LIO-Localization-SC-QN/blob/master/LICENSE)
均明确禁止商业用途并要求署名/同方式共享。`package.xml` 中的 `TODO` 不是有效的
替代许可证。本报告不是法律意见；直接复制前必须得到用途确认，必要时向作者取得
其他授权。

可选路线：

- **非商业研究、接受 copyleft/SA**：固定 commit，以独立 third-party 包保留完整
  LICENSE/NOTICE，并记录改动；技术移植成本最低。
- **存在商业或闭源可能（推荐按此保守设计）**：不复制两个主仓库、Scan Context
  或 Quatro 源码；仅根据论文/公开接口做独立实现，或选择许可证兼容的实现。
  Nano-GICP MIT 与系统 GTSAM 可独立评估。若仍必须使用 Quatro，先取得作者授权。
- **ROS1 bridge/容器运行上游**：许可证问题仍存在，还会增加 ROS1、bridge、bag 和
  TF owner 复杂度，不推荐。

## 10. 建议 package 与文件清单

实际实现不移动 `src/FAST_LIO_Hesai`，在当前 workspace 新增：

```text
src/
├── a2w_fastlio_interfaces/
│   └── msg/LocalizationStatus.msg
├── a2w_fastlio_common/
│   ├── include/a2w_fastlio_common/{common,place_recognition,registration}/...
│   ├── src/{scan_context_manager,quatro_registration,nano_gicp_registration}.cpp
│   └── test/...
├── a2w_fastlio_map/
│   ├── include/a2w_fastlio_map/{map_manager,map_loader,map_io,...}.hpp
│   ├── src/...
│   └── test/...
├── a2w_fastlio_mapping/
│   ├── include/a2w_fastlio_mapping/{keyframe_manager,local_map_builder,
│   │   loop_validator,pose_graph_optimizer,map_odom_manager,mapping_backend}.hpp
│   ├── src/...
│   ├── config/mapping.yaml
│   └── test/...
└── a2w_fastlio_localization/
    ├── include/a2w_fastlio_localization/{localization_manager,
    │   localization_monitor,local_map_selector,map_matcher,relocalizer}.hpp
    ├── src/...
    ├── config/localization.yaml
    └── test/...
```

现有 `src/a2w_fastlio2_bringup` 后续新增：

```text
launch/mapping.launch.py
launch/localization.launch.py
launch/visualization.launch.py
config/frame_policy.yaml
rviz/...
```

建议的新增工程级文件：

```text
docs/architecture.md
docs/map_bundle_v1.md
docs/third_party_sources.md
scripts/save_map_bundle.sh
scripts/check_slam_interfaces.sh
tests/test_tf_owner.sh
```

### 10.1 预计修改的现有文件

| 文件 | 预计改动 | 阶段 |
| --- | --- | --- |
| 根 `README.md` | 增加 Mapping/Localization 使用、状态、Map Bundle 和许可证说明 | 各阶段 |
| 根 `.gitmodules` | 仅在许可证确认且采用 submodule 方案时增加算法依赖 | Stage 2/3 |
| 根 `.gitignore` | 忽略完整 Map Bundle 运行产物、保留示例 metadata | Stage 5 |
| `a2w_fastlio2_bringup/CMakeLists.txt` | 安装新增 launch/config/rviz | Stage 1+ |
| `a2w_fastlio2_bringup/package.xml` | 声明新运行依赖 | Stage 1+ |

预计不修改：

```text
src/FAST_LIO_Hesai/src/laserMapping.cpp
src/FAST_LIO_Hesai/src/preprocess.cpp
src/FAST_LIO_Hesai/include/ikd-Tree/**
src/a2w_fastlio2_bringup/config/jt128.yaml
机器人端 Hesai Driver、网络和雷达配置
```

如果 Stage 1 证明缺少不可替代的 frame 参数化接口，必须单独报告证据并获得确认，
才允许对 FAST_LIO_Hesai 做最小改动。

## 11. Stage 0～9 实施计划与检查点

| Stage | 交付 | 通过条件 |
| --- | --- | --- |
| 0 | 本报告、接口/许可证/依赖基线 | 用户批准架构；确认用途许可证策略；联机前不伪造 runtime 数据 |
| 1 | ROS2 `FrontendFrame` 同步 + KeyFrameManager | 离线消息测试覆盖 stamp、平移/旋转/时间阈值；稳定输出 keyframe |
| 2 | Scan Context + descriptor DB + Top-K | 固定数据集验证 Top-K 排名、score、yaw；不接 GTSAM |
| 3 | LocalMapBuilder + Quatro + Nano-GICP + Validator | 离线 true/false candidate；输出相对位姿和拒绝原因 |
| 4 | GTSAM Prior/Odom/Loop + iSAM2 + map→odom | 单元测试小型 pose graph；运行时 TF owner 唯一且无跳变/环 |
| 5 | optimized map 重建 + Map Bundle V1 | save/load round-trip、manifest/hash、重启可加载；Mapping V1 完成 |
| 6 | Map Loader + local selector + map matcher | 只定位不扩图；低频 correction、高频 global pose |
| 7 | LocalizationMonitor 状态机 | 故障注入验证 INITIALIZING/LOCALIZED/DEGRADED/LOST/RELOCALIZING |
| 8 | LOST 后 Scan Context Top-K 全局重定位 | 较差初值/搬动数据离线恢复；错误候选被 validator 拒绝 |
| 9 | Mapping/Localization/Relocalization 联合实机验收 | 闭环、重启加载、错误初值、重复结构、性能与 TF 全通过 |

每个 Stage 单独提交，顺序执行：

```text
build → unit/offline test → runtime check（有机器人时）→ review → 下一 Stage
```

新功能使用独立分支；建议分支名：

```text
feature/slam-localization-backend
```

## 12. 风险清单

| 风险 | 影响 | 缓解/决策门 |
| --- | --- | --- |
| CC BY-NC-SA / GPL / Quatro 许可证不一致 | 不能合法用于商业或闭源交付 | Stage 1 前确认用途；保守采用净室实现或取得授权 |
| 当前 frame 名与目标不一致 | TF 断树、多 parent、Nav2 不兼容 | 联机抓完整 TF；frame 全参数化；自动测试 TF owner |
| LiDAR/IMU 输入 frame 未知 | 错误外参/错误可视化 | 只从实机 header/TF 确认，不猜测 |
| 上游仅 Top-1 | 重复厂区误匹配、无法全局重定位 | Scan Context API 从第一版就返回 Top-K |
| 上游 validator 太弱 | 错误回环破坏整图 | 分阶段启用 convergence+fitness+separation，再加 overlap/多帧一致性 |
| Quatro source/target 尺度差异 | 粗配准失败或耗时过大 | current/candidate local cloud，点数和范围门限 |
| TEASER++ 缺失 | Quatro 无法编译 | 固定兼容版本，先在独立包验证，不污染前端依赖 |
| GTSAM 4.1.1 ABI/CMake 差异 | Stage 4 编译/链接失败 | 最小 target 编译测试，显式 target link，不升级 Eigen/PCL |
| 重计算阻塞前端 callback | 丢帧、延迟、内存增长 | worker/callback group、有界队列、性能 topic/log |
| 前端累计 PCD 不是优化地图 | 回环后地图仍重影 | Map Bundle global map 必须由 keyframe + optimized pose 重建 |
| Map Bundle 版本漂移/中断写入 | 地图无法加载或静默错误 | schema version、manifest/hash、原子写、round-trip test |
| `extrinsic_est_en=true` | 地图会依赖运行期间估计状态 | 保持当前基线；metadata 记录配置与版本；不擅自修改 |
| 无当前实机数据 | 无法确认频率、QoS 端到端和 frame | 所有未测值显式标记；Stage 1 runtime gate 再补证据 |

## 13. 验收方法

### 13.1 每阶段自动验收

- `colcon build --symlink-install` 全 workspace 通过。
- `colcon test` 和独立算法单元测试通过。
- Map Bundle save/load round-trip，关键帧数、pose、descriptor、hash 一致。
- TF 测试保证 Mapping 与 Localization launch 互斥 owner；任何时刻只有一个
  `map→local_odom` publisher。
- Topic 同步测试覆盖相同 stamp、乱序、缺包、回退时间和慢消费者。
- 配准测试保存 source/target/coarse/fine 及指标，确保错误候选不会仅因
  `hasConverged()` 被接受。

### 13.2 Mapping 实机验收

- 闭环路线完整运行，记录 candidate Top-K、SC score、Quatro、Nano-GICP、validator
  和 loop factor。
- 对比回环前后首尾 pose 误差、轨迹连续性、地图重影和 `map→local_odom` 跳变。
- 保存 Map Bundle，停止后重新加载并重建相同 optimized map。
- FAST_LIO `/Odometry` 高频链路不能被低频 loop/PGO 阻塞。

### 13.3 Localization 实机验收

- 新进程加载前一天 Map Bundle，不增加 keyframe DB、不扩 Pose Graph、不改地图。
- 持续输出 global pose/odom/path 和唯一 TF 链。
- 记录 map match frequency、fitness、overlap、correspondence、correction jump、TF age。

### 13.4 Relocalization 与重复结构验收

- 系统重启、较差初值、人为错误 `map→odom`、条件允许时搬动机器人。
- 观察 LOST → RELOCALIZING → LOCALIZED，并验证 Top-K 各候选的完整指标。
- 至少包含一个 true-positive loop 和一个重复结构 false-positive candidate；错误候选
  必须被拒绝，必要时再启用 multi-frame confirmation/robust loop factor。

### 13.5 性能记录

Mapping 和 Localization 均记录 CPU、RAM、keyframe 数、Scan Context/Quatro/
Nano-GICP/PGO/map-match 耗时、队列深度与更新频率。性能阈值需在 ThinkBook 和
真实 JT128 数据首次基准后确定，本报告不凭空给出数值。

## 14. 明确不在本轮范围

- 正式持久化 Submap Manager
- Nav2
- GNSS / RTK / AprilTag / HBA
- PGO 回写 FAST_LIO ESIKF
- ikd-tree 重构
- 室外定位
- 修改机器人系统、网络、雷达目标地址、Hesai Driver 或宇树自带 SLAM 服务

## 15. 进入 Stage 1 前的确认项

1. 本项目是否严格为非商业研究用途；若不是，采用许可证保守路线并暂停复制
   Scan Context/Quatro/两个主仓库代码。
2. 批准 package 边界、Map Bundle V1 和 Mapping/Localization 分 launch 设计。
3. 下次连接机器人后采集一次 Topic header、频率、QoS 和完整 TF 快照，决定 frame
   policy；在此之前可先做纯离线 KeyFrameManager 单元测试。

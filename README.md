# A2W Fast-LIO2

Unitree A2-W Pro + Hesai JT128 的 PC 端 FAST-LIO2 ROS 2 Humble 工作区。

项目使用禾赛官方适配 JT128 的
[`FAST_LIO_Hesai`](https://github.com/HesaiTechnology-Spatial-Perception/FAST_LIO_Hesai)
ROS2 分支，不使用普通原版 FAST_LIO。上游源码以固定提交的 Git submodule 管理，
`ikd-Tree` 会递归初始化。

## 适用环境

- Ubuntu 22.04
- ROS 2 Humble
- x86_64
- `rmw_cyclonedds_cpp`
- 机器人网络 `192.168.123.0/24`
- 默认 DDS peer：`192.168.123.164`
- 输入话题：`/lidar_points`、`/lidar_imu`

本仓库只负责 PC 端 FAST-LIO2。脚本不会登录或修改 A2-W 主机，不会调整机器人
网络、雷达目标地址或驱动参数，也不会停止宇树自带 SLAM 服务。

## 下载与安装依赖

```bash
git clone --recurse-submodules \
  ssh://git@ssh.github.com:443/AaronYanC/A2W_FastLio2.git
cd A2W_FastLio2
./scripts/bootstrap.sh
```

如果已经普通 clone，`bootstrap.sh` 也会自动初始化全部 submodule。该脚本使用
`rosdep` 安装 `src/` 下 ROS 包声明的依赖；系统需要预先安装 ROS 2 Humble、
`git`、`python3-rosdep` 和 `python3-colcon-common-extensions`。

## 编译

```bash
./scripts/build.sh
```

脚本从自身位置解析仓库根目录，因此可以从其他当前目录调用。默认加载
`/opt/ros/humble/setup.bash`，构建结果位于仓库的 `build/`、`install/` 和 `log/`。

## 检查 JT128 输入话题

```bash
./scripts/check_lidar_topics.sh
```

该命令只读取 ROS 2 graph，检查 `/lidar_points` 和 `/lidar_imu` 的 publisher、
subscriber、类型及 QoS，不会启动、停止或修改机器人端服务。

## 在线运行 FAST-LIO2

不保存地图：

```bash
./scripts/run_fastlio_jt128_pc.sh
```

不启动 RViz：

```bash
./scripts/run_fastlio_jt128_pc.sh rviz:=false
```

## Stage 1：关键帧接入

Stage 1 在现有 Hesai FAST-LIO2 前端旁路启动 ROS2 Mapping ingress：

```bash
./scripts/run_fastlio_jt128_stage1.sh
```

不启动 RViz：

```bash
./scripts/run_fastlio_jt128_stage1.sh rviz:=false
```

该节点 ExactTime 同步现有 `/Odometry` 与 `/cloud_registered_body`，根据平移、旋转
或最大时间间隔生成关键帧，并发布：

```text
/mapping/keyframe_odom
/mapping/keyframe_cloud
```

默认阈值位于 `a2w_fastlio_mapping/config/mapping.yaml`：1.0 米、10 度、2.0 秒。
Stage 1 只生成关键帧，尚不包含 Scan Context、回环、Quatro、Nano-GICP、GTSAM、
优化地图、Localization、Relocalization 或 `map→odom`。该节点自身不发布 TF。

如需同时保存前端的未优化累计 PCD，可显式启用：

```bash
./scripts/run_fastlio_jt128_stage1.sh \
  save_frontend_pcd:=true \
  frontend_map_file:=/data/maps/frontend_map.pcd
```

## Stage 2：Scan Context / Top-K（离线实现）

Stage 2 在 `a2w_fastlio_common` 中提供 ROS 无关的抽象接口：

```text
PlaceRecognition → ScanContextPlaceRecognition
DescriptorIndex  → ScanContextIndex
```

接口支持参数化极坐标描述子、旋转 yaw hint、近期关键帧排除和稳定 Top-K 排名。
重复结构会保留为多个候选，Scan Context 不会直接确认回环或全局位置。默认参数位于：

```text
src/a2w_fastlio_mapping/config/scan_context.yaml
```

这些参数来自固定上游算法的离线初值，并未通过 A2W/JT128 实机调参。Stage 2 当前状态：

```text
Offline implementation and verification complete; JT128 hardware validation pending.
```

## Stage 3：共享粗到细配准（离线实现）

Stage 3 在 `a2w_fastlio_common` 内提供统一接口和一份共享实现：

```text
CoarseRegistration -> QuatroRegistration
FineRegistration   -> NanoGicpRegistration
RegistrationResult -> MatchValidator -> RegistrationPipeline
```

Mapping、Localization 与 Global Relocalization 上层只依赖抽象接口；Quatro、TEASER++
和 Nano-GICP 类型不会出现在上层头文件中。`LocalMapBuilder` 从附近关键帧临时构图，
当前没有持久化 Submap Manager。流水线以统一最近邻口径计算 fitness、overlap 和匹配数，
并要求候选间隔与几何证据同时通过，Scan Context Top-1 或单独的 `hasConverged()` 都不能
确认回环。

离线默认参数位于：

```text
src/a2w_fastlio_mapping/config/registration.yaml
src/a2w_fastlio_mapping/config/loop_validation.yaml
```

这些数值只通过合成数据测试，并非 A2W/JT128 实机调参结果。Stage 3 算法层尚未改变
现有 FAST-LIO 前端，也尚未发布回环、优化 TF 或优化地图。

```text
Offline implementation and verification complete; JT128 hardware validation pending.
```

## Stage 4：GTSAM/iSAM2 位姿图（离线实现）

`a2w_fastlio_mapping` 使用系统 GTSAM 4.1.1，增量加入首帧 Prior、相邻关键帧
Odometry factor 和已通过 Stage 3 验证的 Loop factor。闭环噪声支持 Huber/Cauchy robust
kernel；未验证、越界、同 ID 或非有限约束在改变图状态前被拒绝。

坐标约定统一为 `map_T_body`：节点值表示 body 在 map 中的位姿，Between factor 的测量
表示 `from_T_to`。GTSAM Pose3 对角噪声顺序为旋转 x/y/z（弧度），再平移 x/y/z（米）。
图优化只更新后端关键帧位姿，不向 FAST-LIO 的 ESIKF 或 ikd-tree 回写。

`LoopPipeline` 负责 Top-K 候选、临时 Local Map、共享粗到细配准、每窗口最多一个闭环和
有界输入队列。默认参数位于 `a2w_fastlio_mapping/config/pose_graph.yaml`，均为离线初值。
Stage 4 本身不发布 `map→camera_init`、optimized path 或地图预览；这些输出由 Stage 5
ROS 适配层负责。

```text
Offline implementation and verification complete; JT128 hardware validation pending.
```

## Stage 5：优化输出与全局 TF 所有权（离线实现）

完整 Mapping 模式由以下入口组合现有 Hesai 前端、关键帧接入和 Mapping 后端：

```bash
ros2 launch a2w_fastlio2_bringup mapping.launch.py
```

后端发布 `/mapping/optimized_odom`、`/mapping/optimized_path`、
`/mapping/optimized_map_preview` 和 `/mapping/registration_status`。坐标链保持
`map → camera_init → body`；`map → camera_init` 发布前先通过
`/a2w_fastlio/global_tf_owner` 完成可配置观察窗口。发现同一坐标变换的其他 active owner
后冲突状态会锁存，本进程停止发布全局 TF 并给出 fault status。

关键帧回调只写入有界队列；独立 worker 通过 `DefaultAlgorithmSuite` 的抽象服务运行
`PlaceRecognition → CoarseRegistration → FineRegistration → RegistrationResult`，再将已验证
闭环送入 GTSAM。具体 Scan Context、Quatro、Nano-GICP 类型仍封装在
`a2w_fastlio_common`，Mapping 上层没有直接实现依赖。

`/mapping/optimized_map_preview` 是有体素降采样和最大点数限制的 RViz/调试产物，默认
QoS 为 reliable、transient-local、keep-last 1。工程不会创建
`/mapping/optimized_map` 高密度长驻 Topic。完整优化点云只保留在后端内存，Stage 6 将通过
`a2w_fastlio_map` 写入 Map Bundle 的 `global_map.pcd`。

参数位于：

```text
src/a2w_fastlio_mapping/config/mapping_topics.yaml
src/a2w_fastlio_mapping/config/pose_graph.yaml
```

当前只通过 synthetic keyframe、变换一致性、地图点数边界、QoS 和双 owner 冲突测试；
Topic 频率、真实 frame/timestamp、实时性能及整机 TF 树尚未实机验证。

```text
Offline implementation and verification complete; JT128 hardware validation pending.
```

## Stage 6：统一 Map Bundle V1（离线实现）

推荐用项目根目录解析 launcher 启动完整 Mapping：

```bash
./scripts/run_a2w_mapping.sh
```

建图过程中或停止前，请通过后端快照保存统一地图包（`output_path` 必须位于配置的
`bundle_root` 内）：

```bash
ros2 service call /mapping/save_map_bundle \
  a2w_fastlio_msgs/srv/SaveMapBundle "{output_path: factory_map}"
```

输出目录默认为 `maps/factory_map/`，其中包含 `metadata.yaml`、SHA-256 manifest、
关键帧 PCD、优化位姿、Scan Context 描述子、配置快照和完整 `global_map.pcd`。写入过程
使用同文件系统 sibling staging、生产 Reader 自验证、旧目录备份 rename 和失败回滚；
Localization 后续只读加载相同格式。检查现有地图包：

```bash
ros2 run a2w_fastlio_map map_bundle_inspect maps/factory_map
```

Map Bundle 的 `creation_status` 为 `offline_verified`，
`hardware_validation_status` 为 `pending`；这只说明格式与离线数据链通过验证，不表示地图
来自 JT128 实机。

## Stage 7：已有 Map Bundle 定位（离线实现）

Localization 以只读方式校验并加载 Map Bundle V1，根据当前全局预测位姿选择附近关键帧
临时构建 Local Map，再通过公共接口执行 `Quatro → Nano-GICP → MatchValidator`。低频匹配
更新 `map → camera_init` 校正，高频 FAST-LIO 里程计持续输出 `map` 坐标系下的 pose、odom
和 path。Mapping 与 Localization 使用不同启动模式，同一模式只允许一个全局 TF owner。

```bash
./scripts/run_a2w_localization.sh maps/factory_map
```

主要输出为 `/localization/pose`、`/localization/odom`、`/localization/path` 和
`/localization/registration_status`。参数位于：

```text
src/a2w_fastlio_localization/config/localization.yaml
src/a2w_fastlio_localization/config/map_matching.yaml
src/a2w_fastlio_localization/config/localization_topics.yaml
```

当前验证仅包括合成 Map Bundle、已知变换配准、低频校正/高频传播、Topic frame/stamp/QoS、
TF owner 和 Map Bundle 内容不变性。真实 JT128 Topic、帧、时间戳、匹配稳定性与性能仍待
集中实机验收。

```text
Offline implementation and verification complete; JT128 hardware validation pending.
```

## Stage 8：LocalizationMonitor（离线实现）

纯 C++ `LocalizationMonitor` 根据配准证据、连续成功/失败次数、修正年龄和重定位超时维护：

```text
INITIALIZING → LOCALIZED → DEGRADED → LOST → RELOCALIZING → LOCALIZED
```

`LOST` 不能被普通局部匹配直接恢复，必须显式进入 `RELOCALIZING` 并通过强匹配或多帧一致
匹配门槛。`LOST/RELOCALIZING` 时节点停止发布依赖旧修正的定位 pose/odom/path 与
`map → camera_init`；失败匹配不会刷新修正年龄。可观察状态发布在 `/localization/status`，
包含状态、原因、计数器、修正年龄、候选 ID、配准指标以及固定为 `true` 的
`hardware_validation_pending`。

所有状态转换参数位于：

```text
src/a2w_fastlio_localization/config/relocalization.yaml
```

参数目前只经过确定性状态机测试与合成 ROS2 集成测试，尚未使用 JT128 调参。

```text
Offline implementation and verification complete; JT128 hardware validation pending.
```

## 建图并保存 PCD

```bash
./scripts/run_fastlio_jt128_mapping.sh
```

完成建图后在运行该 launch 的终端按一次 `Ctrl+C`，让 FAST-LIO2 正常退出并写入：

```text
maps/jt128_map.pcd
```

保存配置使用 `pcd_save.leaf_size: 0.1` 米，避免长时间建图产生过大的 PCD。
如需原始密度，可在 `src/a2w_fastlio2_bringup/config/jt128.yaml` 中改为 `0.0`
后重新编译。地图属于运行产物，默认不会提交到 Git。

自定义地图位置：

```bash
A2W_MAP_FILE=/data/maps/site_a.pcd ./scripts/run_fastlio_jt128_mapping.sh
```

## CycloneDDS 自动配置

运行脚本先检查 PC 的全局 IPv4：

```bash
ip -o -4 addr show scope global
```

它优先选择与 DDS peer 同一直连子网的接口，避免 Clash/TUN 路由覆盖机器人网卡；
没有直连匹配时才回退到 `ip -4 route get`。随后在 `log/fast_lio_runtime/` 生成本次
运行使用的 CycloneDDS XML。仓库配置中不保存固定 PC 地址。

多网卡或特殊网络环境可显式覆盖：

```bash
A2W_NETWORK_INTERFACE=enp49s0 \
A2W_PC_IP=192.168.123.100 \
A2W_DDS_PEER=192.168.123.164 \
./scripts/run_fastlio_jt128_pc.sh
```

常用覆盖变量：

| 变量 | 默认值 | 用途 |
| --- | --- | --- |
| `A2W_DDS_PEER` | `192.168.123.164` | CycloneDDS peer 和路由探测目标 |
| `A2W_NETWORK_INTERFACE` | 自动探测 | PC 连接机器人交换机的网卡 |
| `A2W_PC_IP` | 自动探测 | 该网卡的 PC IPv4 地址 |
| `A2W_MAP_FILE` | `maps/jt128_map.pcd` | 保存地图的输出位置 |
| `A2W_RUNTIME_DIR` | `log/fast_lio_runtime` | 运行时 DDS 配置目录 |
| `A2W_ROS_SETUP` | Humble 系统 setup | ROS 环境脚本 |
| `A2W_INSTALL_SETUP` | 本工作区 install setup | 编译后环境脚本 |
| `A2W_PYTHON_EXECUTABLE` | `/usr/bin/python3` | ROS CMake/消息生成使用的 Python |

## 目录结构

```text
.
├── config/                       # PC 端 CycloneDDS 模板
├── maps/                         # 运行时地图；Git 忽略 PCD
├── scripts/                      # 初始化、编译、检查和运行入口
├── src/
│   ├── FAST_LIO_Hesai/           # 禾赛官方固定版本 submodule
│   ├── a2w_fastlio_common/        # ROS 无关的共享后端类型和算法接口
│   ├── a2w_fastlio_map/           # Map Bundle 持久化与只读加载
│   ├── a2w_fastlio_mapping/       # 关键帧、回环、PGO 与优化地图
│   ├── a2w_fastlio_localization/  # 已有 Map Bundle 实时定位
│   ├── a2w_fastlio_msgs/          # Mapping/Localization ROS2 接口
│   └── a2w_fastlio2_bringup/     # A2-W JT128 配置和 launch
└── tests/                        # 可迁移性与配置行为测试
```

## 测试

```bash
./tests/run_tests.sh
```

测试覆盖运行时 DDS 配置生成、从任意当前目录启动、地图路径传递以及仓库本机路径
清理。完整交付还应运行 `./scripts/build.sh`。

## 算法与许可证边界

FAST-LIO2 仍是唯一前端。当前分支已有离线 Scan Context 与粗到细配准算法层，
但在后续 Stage 接入回环因子、GTSAM 和运行时输出前，不应将它描述为已完成全局
优化的建图系统。

雷达—IMU 外参目前沿用配置中的单位变换，仅适合作为当前链路基线。正式测量或导航
前应使用厂商参数或标定结果确认，但该工作不由本仓库脚本自动执行。

Scan Context 核心来自固定版本 `engcang/scancontext_tro`，上游声明为
CC BY-NC-SA 4.0。它与本项目原创代码的许可证边界、来源和适配内容记录在
`third_party/THIRD_PARTY_NOTICES.md` 和对应 `UPSTREAM.md` 中。Quatro 与 PMC 还带有 GPL
条款，Quatro 上游元数据存在 GPL/CC BY-NC-SA 许可信号不一致。因此当前仓库
不能整体笼统声明为 Apache-2.0，也不能在未完成许可证审查前声称允许商业交付。

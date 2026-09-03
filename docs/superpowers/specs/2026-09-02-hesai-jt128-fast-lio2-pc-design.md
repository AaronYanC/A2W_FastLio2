# Hesai JT128 + FAST-LIO2 PC 端设计

## 目标

在 ThinkBook（Ubuntu 22.04、ROS 2 Humble、x86_64）上的当前工作区中，
完成 Hesai 官方 `FAST_LIO_Hesai`
ROS2 分支的可复现构建、JT128 配置检查和无输入启动验证。

本阶段的完成标准是：`fast_lio` 包成功构建并能被 ROS 2 发现；JT128
参数通过仓库自带检查；节点可在 CycloneDDS 环境下启动并创建对
`/lidar_points` 和 `/lidar_imu` 的订阅。没有雷达数据时，不要求建图输出。

## 范围边界

- PC 只运行 FAST-LIO2；本阶段不在 PC 上运行 Hesai Driver。
- 不修改或停止 `192.168.123.162`、`192.168.123.164` 上的服务。
- 不修改机器人网络、雷达目标 IP、雷达端口或雷达参数。
- 不停止宇树自带 SLAM 服务。
- 不把 Hesai ROS2 Driver 合并进 FAST-LIO2 工作区。
- Nav2 不属于本阶段实现范围。

后续数据链路由独立阶段处理：`.164` 上的 Hesai Driver 发布
`/lidar_points`、`/lidar_imu`，再通过 DDS 发送到 PC。

## 当前状态与阻塞原因

- `src/FAST_LIO_Hesai` 是 Hesai 官方仓库的 `ROS2` 分支，当前提交与
  本地记录的 `origin/ROS2` 一致。
- 现有两次 `colcon` 构建都在 CMake 配置阶段失败，因为
  `include/ikd-Tree/ikd_Tree.cpp` 不存在。
- `.gitmodules` 已将该目录声明为 `ikd-Tree` 子模块，并锁定到提交
  `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`；问题是子模块未初始化。
- ROS 2 Humble、PCL、Eigen、`pcl_ros`、`pcl_conversions` 和
  `rmw_cyclonedds_cpp` 已安装。
- 现有 CycloneDDS XML 固定到旧接口地址 `172.20.10.11`，不能用于
  当前 `enp49s0 = 192.168.123.100/24` 链路。因此验证命令只显式选择
  CycloneDDS RMW，不复用该旧 XML。

## 实施设计

### 源码与依赖

在 `src/FAST_LIO_Hesai` 内执行递归子模块初始化，取得仓库锁定的
`ikd-Tree` 版本。不得手工复制其他 FAST-LIO 仓库中的 ikd-Tree，也不得
更改子模块指针。

初始化后检查目标源文件存在，并确认 FAST-LIO_Hesai 主仓库没有意外修改。

### 构建

从工作区根目录加载 `/opt/ros/humble/setup.bash`，使用：

```bash
colcon build --packages-select fast_lio --symlink-install
```

只构建 `fast_lio`，不加载或构建其他 Hesai Driver 工作区。若旧构建缓存
导致与源码状态不一致，仅清理 `fast_lio` 对应的构建产物，不删除其他目录。

### JT128 参数

使用官方 `config/jt128.yaml`，关键契约为：

- `common.lid_topic: /lidar_points`
- `common.imu_topic: /lidar_imu`
- `common.time_sync_en: false`
- `common.imu_gyr_unit: auto`
- `preprocess.lidar_type: 2`
- `preprocess.scan_line: 128`
- `preprocess.timestamp_unit: 0`

本阶段不修改 `mapping.extrinsic_T` 和 `mapping.extrinsic_R`。官方单位阵仅用于
软件链路验证，不代表 A2-W Pro 的实测雷达—IMU 外参；在正式建图前必须从
机器人资料或标定结果确认外参。

### CycloneDDS 与启动验证

验证进程显式设置：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

不设置现有旧网段 `CYCLONEDDS_URI`。随后验证：

1. `ros2 pkg prefix fast_lio` 能找到安装包。
2. 仓库自带 `check_config.py` 对 JT128/ROS2 配置检查通过。
3. `mapping_jt128.launch.py` 在 `rviz:=false` 下能够启动。
4. 启动日志显示输入话题、`lidar_type=2`、`scan_line=128` 等参数正确。

由于本阶段没有 `/lidar_points` 与 `/lidar_imu`，节点等待输入属于预期行为，
不是验证失败。验证结束时只终止本次 PC 端测试进程。

## 错误处理

- 子模块下载失败时保留现有源码并报告网络错误，不使用来源不明的替代代码。
- 构建失败时保留完整 `colcon` 日志，根据首个编译错误处理，不修改机器人端。
- 若配置检查失败，先对比已安装文件和源码文件，避免编辑错误副本。
- 若 CycloneDDS 初始化失败，检查 PC 本地 RMW 环境；不改机器人 DDS 配置。

## 验证与交付

交付证据包括：子模块状态、`colcon` 成功摘要、ROS 包前缀、配置检查输出和
受控启动日志。最终说明会区分“PC 软件已就绪”和“尚未打通 `.164` 数据链路”，
不会把无传感器输入误报为完成实机建图。

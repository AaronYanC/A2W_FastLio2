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
│   └── a2w_fastlio2_bringup/     # A2-W JT128 配置和 launch
└── tests/                        # 可迁移性与配置行为测试
```

## 测试

```bash
./tests/run_tests.sh
```

测试覆盖运行时 DDS 配置生成、从任意当前目录启动、地图路径传递以及仓库本机路径
清理。完整交付还应运行 `./scripts/build.sh`。

## 算法边界

FAST-LIO2 提供紧耦合激光—惯性里程计和增量地图，但当前工程不包含回环检测、
位姿图全局优化或多会话地图融合。大范围或长时间运行仍可能累积漂移；若后续需要
全局一致地图，应单独集成回环/图优化方案，并在不影响机器人自带服务的前提下验证。

雷达—IMU 外参目前沿用配置中的单位变换，仅适合作为当前链路基线。正式测量或导航
前应使用厂商参数或标定结果确认，但该工作不由本仓库脚本自动执行。

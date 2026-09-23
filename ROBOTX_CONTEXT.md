# RobotX 项目上下文（供本地 Codex 使用）

> 更新时间：2026-09-18  
> 用途：为本地 Codex 提供 RobotX 项目的长期背景。执行任何修改前，应先阅读本文，并结合仓库中的实际代码、设备输出、PX4 参数文件和飞行日志重新核验。  
> 重要说明：本文整理自历史聊天，不等同于最新实机状态。标记为“待确认”的内容不得自行假设为真。

## 1. 使用规则与状态标记

本文使用以下状态：

- **已确认**：历史操作、命令输出或测试结果已明确支持。
- **曾观察到**：某次调试中出现过，当前是否仍然成立需要复核。
- **计划/建议值**：讨论过但没有证据表明已写入设备或完成测试。
- **待确认**：信息缺失、存在冲突，或需要重新上机检查。

本地 Codex 在处理本项目时必须遵守：

1. 先确认操作对象：Windows 主机、VMware Ubuntu、UAV 机载 Jetson、USV Orin、Pixhawk 飞控或仿真环境。
2. 不得把 UAV 的 ROS 2 Humble 环境与 USV Orin 的 ROS 1 Noetic 环境混为一台机器或同一套工作空间。
3. 修改 PX4 参数、网络配置、ROS 工作空间或启动项前，先读取现状并保存备份/差异。
4. 不得自动解锁、自动取消 Kill、绕过遥控接管或降低失控保护。Offboard 脚本保持“人工解锁、可由遥控退出、Kill 有效”。
5. 电机测试前必须明确螺旋桨是否拆除；实飞前必须确认场地、人员、遥控链路、定位源和失控策略。
6. 不要把“脚本已编写”等同于“实机完整测试已通过”；结论需要日志、ULog 或视频证据。
7. 对不确定信息明确写“待确认”，不要补造版本号、接口名、参数值或测试结论。

## 2. 项目概况

- 赛事：国际海事机器人大赛 **RobotX**（不是 RoboCup）。
- 用户基础：从零学习机器人、PX4、ROS、UAV/USV 调试。
- 总体目标：
  - 完成 PX4 学习、Gazebo 仿真、实机调试与文档化。
  - 完成 UAV 室内动捕定位、Offboard 飞行及室外 GPS 飞行。
  - 完成 USV 首次上电检查、基本校准、通信与下水测试。
  - 建立私有 GitHub 仓库，保存环境配置、启动流程、代码和测试记录。

## 3. UAV 硬件平台

### 3.1 已知配置

| 类别 | 型号/状态 | 可信度与备注 |
|---|---|---|
| 机架 | Holybro X500 四旋翼 | 已确认 |
| 飞控 | Pixhawk 6X + Mini Baseboard | 已确认 |
| 电源/分电 | Holybro PM03D 数字电源/分电模块 | 已确认 |
| 电机 | T-Motor MN3508 | 已确认 |
| 电调 | T-Motor Air 40A | 已确认 |
| 螺旋桨 | P12×4 | 已确认；电机方向曾核对与图一致 |
| 电池 | 6S2P 10000 mAh ×2 | 已确认；仅一块可用时容量按 10000 mAh 计算 |
| 电池电压 | 24.967 V | 曾用万用表两次测得相同读数 |
| 遥控器 | RadioMaster TX16（ELRS 版本） | 已确认 |
| 接收机 | RadioMaster RP3 ExpressLRS | 已确认 |
| 数传 | Holybro V2 | 已确认；地面端曾设为 master，早期配置失败，后续更多使用机载电脑网口 |
| GPS/罗盘 | 外接 GPS + 外接磁罗盘 | 已确认；设计上仅使用外接磁罗盘 |
| 磁罗盘设备 ID | `MAG1=396809` | 曾设置；当前参数需重新读取确认 |
| GPS 安装偏移 | 未设置 | 历史状态；当前待确认 |

### 3.2 待确认的硬件信息

- 整机实际起飞重量、重心位置、最大安全载荷与续航。
- 两块电池是否并联、独立供电或仅备用，以及线束/保险/防反接方式。
- GPS 精确型号、安装方向、红灯闪烁含义及当前健康状态。
- 光流/激光测距模块的准确型号、供电和连接端口。
- 各串口、CAN、以太网、USB 连接关系及波特率。
- 机载 Jetson 与 USV Orin 是否为两台独立设备；若是同一设备，需说明何时重刷/切换系统。

## 4. USV 与机载计算平台

### 4.1 USV 计算机硬件

- 载板：控元科技 C2401。
- 核心模块：NVIDIA Jetson Orin NX 16 GB。
- 存储：M.2 NVMe SSD 已安装。
- 电源接口：机身上存在两个外观相同的接口；准确输入范围、冗余关系和极性 **待确认**，不得仅凭外观接电。

### 4.2 USV Orin 当前已知软件状态

- 登录用户名：`cwkj`。
- `/opt/ros` 曾显示 `noetic`。
- `which roscore` 曾返回 `/opt/ros/noetic/bin/roscore`。
- `which ros2` 曾无输出。
- 因此该 USV Orin 在当次检查中为 **ROS 1 Noetic 环境**；ROS 2 是否后来安装、是否配置 `ros1_bridge` 均待确认。
- Ubuntu 版本待通过实机命令核验。ROS 1 Noetic 通常对应 Ubuntu 20.04，但不得仅据此推断。
- MAVROS 是否已安装、版本和配置待确认。

### 4.3 USV Orin 上观察到的工作空间

- `calib_ws`
- `cwkj_ws`
- `d435_ws`
- `ego_ws`
- `fast_lio2_ws`
- `lane_detect_ws`
- `livo2_ws`
- `mid_livo2_ws`
- `mid_ws`
- `LIV_handhold/livox_ros_driver`
- `LIV_handhold/livox_ros_driver2`

`cwkj_ws` 曾约 954 MB，其中 `build` 约 246 MB、`devel` 约 128 MB、`src` 约 334 MB。`src` 中曾观察到：

- `api_library`
- `darknet_ros`
- `livox_ros_driver` / `livox_ros_driver2`
- `fast_lio2`
- `mid360`
- `mid_lio2`
- `realsense2_camera`
- `realsense2_description`
- `rplidar_ros`
- `usb_cam`
- `yolov5_ros`

这些目录存在不代表节点当前能够编译或正常运行，需要检查依赖、构建时间和启动文件。

### 4.4 USV 网络

- 更换网卡后，Orin 曾成功连接 Wi-Fi，并可通过 VS Code Remote SSH 连接。
- 与飞控关联的连接名曾为 `PC-VPN`。
- 新增的动态地址连接名曾为 `dhcp-enp8`，作用是通过 DHCP 自动获得 IP，方便连接普通路由器或未知网段设备。
- 两个连接对应的物理网口、路由优先级、静态 IP、飞控地址和是否同时启用均待确认。

## 5. UAV ROS 2、PX4 与仿真环境

### 5.1 开发/刷机环境

- Windows 主机 + VMware Ubuntu 22.04 曾用于 Jetson 刷机。
- SDK Manager 多次失败后最终刷机成功。
- UAV/动捕桥接相关环境曾使用 Ubuntu 22.04 + ROS 2 Humble。
- 常用 ROS 2 环境：
  - `/opt/ros/humble/setup.bash`
  - `~/robotx_ws/install/setup.bash`
- 工具链曾包含 CMake、GCC/G++、Python 3、VS Code Remote SSH。
- PX4 + Gazebo 已安装并进行过仿真/调试；具体 PX4 版本、Git commit、Gazebo 版本和仿真模型版本待确认。

### 5.2 PX4 飞控状态与基础校准

- QGroundControl 曾出现连接异常，后恢复，姿态仪显示正常。
- 加速度计、陀螺仪和磁罗盘校准已完成过。
- 电机方向曾按机架示意图核对。
- 当前校准有效性、PX4 固件升级后是否需要重新校准待确认。

### 5.3 遥控开关映射

| 开关 | 历史用途 | 备注 |
|---|---|---|
| `SF` | Arm/Ready | 上：Armed；下：Ready。当前映射需在 QGC 再核验 |
| `SC` | 模式接管 | Position/Altitude，用于退出 Offboard；具体档位待确认 |
| `SH` | Kill | 松手 Ready，拉住 Kill；必须保持紧急可用 |

## 6. DDS 与动捕定位链路

### 6.1 链路结构

```text
Chingmu 动捕
  -> VRPN tracker: robotx
  -> ROS 2 包 vrpn_dds_bridge / vrpn_to_vio.py
  -> /fmu/in/vehicle_visual_odometry
  -> Micro XRCE-DDS Agent (UDP 8888)
  -> PX4 uXRCE-DDS client
  -> EKF2 融合 EV position/yaw/height
```

### 6.2 动捕与桥接实现

- 动捕系统：Chingmu。
- 原刚体名：`dcruav`。
- 当前使用的刚体名：`robotx`。
- ROS 2 包：`vrpn_dds_bridge`（`rclpy` + `px4_msgs`）。
- 主要脚本：`vrpn_to_vio.py`。
- Launch 文件：`vrpn_vio.launch.py`，曾恢复使用。
- VRPN Python 手写绑定曾通过以下选项源码编译修复：

```bash
-DVRPN_BUILD_PYTHON_HANDCODED_3X=ON
```

- 早期原始轮询率常量：`VRPN_POLL_RATE_HZ=250.0`。
- 坐标单位转换：毫米到米，`scale=0.001`。
- 发布话题：`/fmu/in/vehicle_visual_odometry`。

### 6.3 最新已知桥接参数

```text
vrpn_tracker       = robotx
publish_rate_hz    = 50.0
stale_timeout_s    = 2.0
pos_std_m          = 0.03
att_std_deg        = 2.0
quality            = 100
px4_vio_topic      = /fmu/in/vehicle_visual_odometry
```

### 6.4 曾观察到的数据与 EKF 状态

- `/fmu/in/vehicle_visual_odometry` 曾有有效输出：
  - position 约 `[1.11, 0.73, -0.225]`
  - quaternion 约 `[0.9987, 0.0059, -0.00086, 0.0501]`
- `estimator_aid_src_ev_pos` 曾正常，`test_ratio` 约为 0。
- 早期 EKF flags：
  - `cs_ev_pos=True`
  - `cs_ev_hgt=True`
  - `cs_ev_yaw=False`
  - `cs_baro_hgt=True`
  - `cs_mag_hdg=True`
  - `cs_gnss_pos=False`
- 后续曾观察到：
  - `cs_ev_pos=True`
  - `cs_ev_yaw=True`
  - `cs_ev_hgt=True`
  - `cs_baro_hgt=True`
- 某次 `/fmu/out/vehicle_local_position_v1` 约为 `x=1.653, y=1.203, z=-0.227`。

### 6.5 最新已知 PX4 相关参数

以下值曾用于室内 VIO 配置，必须从当前飞控重新读取确认：

```text
EKF2_EV_CTRL = 11
EKF2_MAG_TYPE = 5
SYS_HAS_MAG = 0
EKF2_HGT_REF = 3
```

注意：上述设置涉及外部视觉、磁罗盘和高度参考，不能直接用于室外 GPS 飞行而不重新评估。

### 6.6 DDS 状态

- 飞控端曾使用：

```text
uxrce_dds_client start -t udp -p 8888
```

- Agent 使用 ROS 2 Humble ARM64 的 Micro XRCE-DDS/Micro ROS Agent 压缩包，曾可单独运行。
- 飞控 `status` 曾出现 `COMMIT LOST`；具体含义、发生频率、是否影响消息收发待结合版本和完整日志确认。
- 关键 ROS 2 话题：
  - `/fmu/in/vehicle_visual_odometry`
  - `/fmu/in/offboard_control_mode`
  - `/fmu/in/trajectory_setpoint`
  - `/fmu/out/vehicle_local_position_v1`
  - `/fmu/out/vehicle_status_v1`
  - `/fmu/out/vehicle_command_ack`

## 7. Offboard 飞行脚本与测试进展

### 7.1 `safe_offboard_mode_test.py`

- 初版用于心跳和起降流程测试。
- 不自动发送 arm。
- 完整实机测试范围与最新代码版本待确认。

### 7.2 `indoor_offboard_flight_test.py`

- 初始为 Position 模式，由用户手动通过 `SF` 解锁，然后脚本请求 Offboard。
- 某次锁定起始位姿：
  - `x≈2.046`
  - `y≈0.132`
  - `z≈-0.217`
  - `yaw≈0.492 rad`
- 收到过 `Offboard acknowledgement: ACCEPTED`。
- 某次切换到 Altitude 时电机停止；需核对当时油门位置、模式切换逻辑和日志，不能简单归因。
- 相关参数曾为：

```text
spoolup_duration = 3 s
COM_OF_LOSS_T = 1.0
COM_OBL_RC_ACT = 0
```

这些 PX4 参数是否仍生效待确认。

### 7.3 `advanced_indoor_offboard_test.py`

- 轨迹：中心与前/后/左/右四点往返，计划循环 2 次。
- 不发送 arm/disarm；保持人工解锁、`SC` 接管、`SH` Kill。
- 某次起始状态：`x=0.136, y=-1.080, z=-0.241, yaw=-3.097 rad`。
- 最新已知脚本参数：

```text
takeoff_height_m                  = 1.0
takeoff_speed_mps                 = 0.15
pre_takeoff_delay_s               = 3.0
takeoff_hold_s                    = 3.0
move_distance_m                   = 0.30
move_speed_mps                    = 0.15
outer_hold_s                      = 1.5
center_hold_s                     = 1.0
pattern_cycles                    = 2
max_horizontal_radius_m           = 0.65
max_height_m                      = 0.80
max_horizontal_speed_mps          = 1.00
max_vertical_speed_mps            = 0.80
max_horizontal_tracking_error_m   = 0.35
max_vertical_tracking_error_m     = 0.35
max_yaw_error_rad                 = 0.436
local_position_timeout_s          = 0.50
timer_period_s                    = 0.10
```

- 状态机：`TAKEOFF_HOLD -> PATTERN -> LAND`。
- 降落命令：`VEHICLE_CMD_NAV_LAND`。
- 完整四点轨迹是否已经在实机无异常完成并保存日志：**待确认**。

### 7.4 已完成/观察到的测试里程碑

- 已完成从准备到起飞、悬停和自动降落的一次基础流程。
- 室内动捕 VIO 可向 PX4 发布并被 EKF 接收。
- Offboard 请求曾被接受。
- QGC 中曾出现：
  - `Armed by RC switch`
  - `logging open`
  - `Disarmed by preflight inaction`（一次）
- 起飞偶发旋转问题尚未形成明确根因结论。

## 8. 室外 GPS、围栏与 RTL 计划

室外飞行目标：不依赖动捕，改用 GPS。以下内容为需求或讨论值，并不代表已经写入飞控：

- 电子围栏目标：水平区域约 20 m × 20 m，高度 15 m。
- 返航/降落讨论值：

```text
RTL_TYPE = 0
RTL_RETURN_ALT = 10 m
RTL_DESCEND_ALT = 5 m
RTL_LAND_DELAY = 3 s
RTL_CONE_ANG = 0
```

- 转弯半径、返航落点、Rally Point、Bench Return Point 的最终设置待确认。
- RTK 的硬件是否具备、基站/移动站型号、差分链路和 PX4 配置待确认。
- 室外飞行前必须重新确认磁罗盘、GPS 健康、Home 点、地理围栏、RTL 行为和定位源切换，不能直接沿用室内禁用磁罗盘/视觉高度参考的设置。

## 9. 常用命令

### 9.1 启动 ROS 2 动捕到 PX4 桥接

```bash
cd ~/robotx_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run vrpn_dds_bridge vrpn_to_vio --ros-args \
  -p vrpn_tracker:=robotx \
  -p publish_rate_hz:=50.0 \
  -p stale_timeout_s:=2.0 \
  -p pos_std_m:=0.03 \
  -p att_std_deg:=2.0 \
  -p quality:=100 \
  -p px4_vio_topic:=/fmu/in/vehicle_visual_odometry
```

若使用 launch 文件，需先确认包中实际 launch 名称和参数：

```bash
ros2 launch vrpn_dds_bridge vrpn_vio.launch.py
```

### 9.2 ROS 2 话题检查

```bash
ros2 topic list | sort
ros2 topic info /fmu/in/vehicle_visual_odometry
ros2 topic echo /fmu/in/vehicle_visual_odometry
ros2 topic echo /fmu/out/vehicle_local_position_v1
ros2 topic echo /fmu/out/vehicle_status_v1
ros2 topic echo /fmu/out/vehicle_command_ack
```

注意：`px4_msgs` 话题是否带 `_v1` 后缀与 PX4/消息版本有关，应以 `ros2 topic list` 为准。

### 9.3 PX4 Shell 中检查 DDS 与 EKF

```text
uxrce_dds_client status
uxrce_dds_client start -t udp -p 8888
listener estimator_status_flags
listener estimator_aid_src_ev_pos
listener vehicle_local_position
```

启动命令不得在已有 client 正常运行时重复执行；先检查 `status`。

### 9.4 检查 USV Orin 系统、ROS 和 MAVROS

```bash
cat /etc/os-release
uname -a
uname -m
ls -la /opt/ros
echo "$ROS_DISTRO"
which roscore
which ros2
rosversion -d 2>/dev/null
dpkg -l | grep -Ei 'ros-.*-(mavros|ros1-bridge)|mavros'
rospack find mavros 2>/dev/null
find /opt/ros -maxdepth 3 -type d -name mavros 2>/dev/null
```

### 9.5 检查网络连接

```bash
ip -br address
ip route
nmcli device status
nmcli connection show
nmcli connection show PC-VPN
nmcli connection show dhcp-enp8
```

不要在远程 SSH 会话中直接删除或重启当前正在承载 SSH 的网络连接。

### 9.6 检查工作空间

ROS 1 catkin 工作空间：

```bash
cd ~/cwkj_ws
source /opt/ros/noetic/setup.bash
[ -f devel/setup.bash ] && source devel/setup.bash
rospack profile
rospack list | sort
```

ROS 2 colcon 工作空间：

```bash
cd ~/robotx_ws
source /opt/ros/humble/setup.bash
[ -f install/setup.bash ] && source install/setup.bash
colcon list
```

## 10. 故障与处理记录

| 问题 | 历史处理/状态 | 当前结论 |
|---|---|---|
| QGC 初期连接异常、无数据输出 | 多次排查后恢复 | 根因未完整记录；再次出现需保存链路、端口和日志 |
| Holybro V2 `AT` 配置及 `pp remote` 转换失败 | 后续改为更多使用机载电脑网口 | 数传最终配置待确认 |
| `import vrpn` 报错 | 源码编译 VRPN Python 手写 3X 绑定后修复 | 已解决过；重装环境需记录依赖版本 |
| `uxrce_dds_client status` 显示 `COMMIT LOST` | Agent 可单独运行，话题曾能收发 | 根因和当前状态待确认 |
| `cs_ev_yaw=False` | 后续曾变为 `True` | 已阶段性改善；需复核当前 EKF 参数与 yaw 源 |
| 起飞偶发旋转 | 未形成明确根因 | 未解决；检查电机/桨、机架、姿态、yaw 源、振动和控制日志 |
| 外置 GPS 红灯闪烁 | 曾被观察 | 含义及健康状态待查型号手册和 QGC GPS 状态 |
| 切换 Altitude 后电机停止 | 某次 Offboard 测试发生 | 待结合 RC 油门、模式、解锁与 ULog 判断 |
| `Disarmed by preflight inaction` | 出现过一次 | 可能是解锁后长时间未起飞；需结合事件时间线确认 |
| QGC 地图版本较旧 | 已提出更新需求 | 当前地图提供商、缓存和版本待确认 |
| USV 上电快速滴滴声 | 已提出排查需求 | 声源及告警含义未确认，首次下水前必须定位 |
| USV 遥控器按键/拨杆功能 | 曾请求说明 | 需要通过通道监视器和厂商配置逐项核验 |
| 光流/激光测距接口 | 曾通过图片询问 | 准确端口、协议和供电待实物/手册确认 |

## 11. 当前待办事项（建议优先级）

### P0：安全与基线记录

1. 区分并标记每台计算机/飞控，记录设备序列号、主机名、系统和用途。
2. 从当前 Pixhawk 导出完整参数文件，并记录 PX4 固件版本和 Git/构建信息。
3. 备份 UAV/USV 工作空间源码、launch 文件、配置、systemd 服务和网络连接信息。
4. 核实遥控 `SF/SC/SH` 映射、模式接管、Kill 和失控策略。
5. 找到 USV 快速滴滴声来源，确认供电接口、电压和极性后再下水。

### P1：环境与通信核验

1. 在 USV Orin 上确认 Ubuntu、ROS、MAVROS、`ros1_bridge` 和所有工作空间的实际状态。
2. 绘制 USV/UAV 实际接线与网络拓扑，标注 IP、端口、协议、波特率和供电。
3. 核对 `PC-VPN` 与 `dhcp-enp8` 的物理接口、路由和使用场景。
4. 复测 Micro XRCE Agent、UDP 8888、PX4 client 和关键话题订阅数量。
5. 把 `vrpn_dds_bridge`、Offboard 脚本和对应 `px4_msgs` 版本提交到私有 Git 仓库。

### P2：UAV 室内闭环与日志

1. 固定一套可复现启动顺序并写入 README。
2. 在拆桨状态检查定位、姿态、模式切换和安全开关。
3. 低风险条件下复测 advanced 四点轨迹，保存 ROS 日志、QGC 事件、PX4 ULog 和视频。
4. 分析起飞偶发旋转和 Altitude 接管后电机停止问题。
5. 对每次测试记录代码 commit、参数文件、起始位姿、结果和异常。

### P3：室外 GPS 飞行

1. 恢复/配置适合 GPS 飞行的磁罗盘、GPS、高度源和 EKF 设置。
2. 设置并实测 Home、RTL、地理围栏、最大高度和失联行为。
3. 从低高度手动 Position 飞行开始，再逐步进入任务/Offboard。
4. 若使用 RTK，先确认硬件、差分链路和定位质量，再进入自动任务。

### P4：USV 首航

1. 完成陆上供电、急停、遥控、推进器方向、舵/差速、通信和传感器检查。
2. 确认防水、浮力、重心、线缆固定和回收方案。
3. 先进行系留/近岸低速测试，再逐步扩大范围。
4. 保存 ROS bag、飞控日志、网络状态和视频。

## 12. 建议的仓库结构

```text
robotx/
├── AGENTS.md
├── ROBOTX_CONTEXT.md
├── README.md
├── docs/
│   ├── hardware/
│   ├── wiring/
│   ├── network/
│   ├── px4-parameters/
│   └── test-reports/
├── ros2_ws/
│   └── src/
│       └── vrpn_dds_bridge/
├── offboard/
│   ├── safe_offboard_mode_test.py
│   ├── indoor_offboard_flight_test.py
│   └── advanced_indoor_offboard_test.py
├── configs/
├── scripts/
└── logs/
```

如果现有仓库结构不同，应优先保留现状并逐步整理，不要未经确认进行大规模移动或重命名。

## 13. 本地 Codex 开始工作时的建议提示词

```text
请先完整阅读 ROBOTX_CONTEXT.md 和 AGENTS.md，再检查仓库实际文件。
先告诉我：
1. 你认为当前任务涉及哪台设备/哪个环境；
2. 文档与仓库之间有哪些一致或冲突之处；
3. 哪些信息需要我上机运行命令确认；
4. 你准备修改哪些文件，以及如何验证。

不要自动改变 PX4 参数、网络配置、开机启动项或遥控安全逻辑。
不要自动解锁或启动电机。所有未确认信息请明确标记为“待确认”。
```

## 14. 后续更新要求

每次重要测试后，应在本文或独立测试报告中补充：

- 日期、地点、操作者和设备。
- Git commit、PX4 版本、参数文件哈希/文件名。
- ROS/系统版本及启动命令。
- 硬件连接和电池状态。
- 预期行为、实际行为、日志与视频路径。
- 异常、处置和下一步。

本文件应作为项目索引，而不是替代原始代码、参数、日志、接线图和厂商文档。

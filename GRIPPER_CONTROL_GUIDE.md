# Orin 通过 PX4 控制 PWM 夹爪

本方案针对本机已经安装的 **PX4 v1.17.0 + px4_msgs v1.17.0**。Orin 上的 ROS 2 节点发送 `VEHICLE_CMD_DO_SET_ACTUATOR`，PX4 把归一化命令转换成某个输出口的 PWM。夹爪控制和姿态、电机控制相互独立，不要求进入 Offboard 模式。

## 1. 控制链路

```text
Orin ROS 2 /gripper/close (Bool)
    -> /fmu/in/vehicle_command
    -> PX4 VEHICLE_CMD_DO_SET_ACTUATOR (187)
    -> Peripheral via Actuator Set 1 (function 301)
    -> 飞控 MAIN/AUX PWM 引脚
    -> PWM 舵机夹爪
```

`true` 表示关闭夹爪，`false` 表示打开夹爪。程序启动时不会自动运动。

### 飞控源码里这条链路怎么工作

本机 PX4 源码在 `/home/quzhankun/robotx/PX4-Autopilot`，关键位置如下：

- `msg/versioned/VehicleCommand.msg`：定义命令 187，`param1..param6` 是 6 路 actuator set，`param7` 是 set index。
- `src/modules/commander/Commander.cpp`：接收 187 并返回 command ACK。
- `src/lib/mixer_module/functions/FunctionActuatorSet.hpp`：当 `param7=0` 时，把有限值的 `param1..param6` 保存为对应输出；其他参数使用 NaN 就不会被改动。
- `src/lib/mixer_module/output_functions.yaml`：规定 `Peripheral_via_Actuator_Set1..6` 的函数号为 301..306。
- `src/lib/mixer_module/mixer_module.cpp`：把 301..306 注册给 `FunctionActuatorSet`。

所以节点只控制被映射为 301 的那一路，不会改电机控制量；PX4 输出驱动再把 `-1..+1` 按该通道的 Min/Max 转为 PWM。ACK 只能证明 Commander 接收了命令，不能证明物理 PWM 和夹爪已经动作。

## 2. 接线（必须先拆桨）

典型三线 PWM 舵机：

- 信号线：接一个确认未被电机占用的 `MAIN` 或 `AUX` 信号脚。
- 电源正：接符合舵机规格的独立 BEC（常见 5–6 V，但必须以夹爪铭牌为准）。
- 地线：舵机地、BEC 地、飞控地必须共地。
- 不要默认用飞控排针给夹爪供电。夹爪堵转电流可能导致飞控掉电或重启。

在知道飞控型号、准备使用的输出口、舵机额定电压与堵转电流之前，不要通电测试。

## 3. QGroundControl 输出设置

下面以空闲的 `MAIN 5` 为例；如果实际接 `AUX 1`，把参数前缀改成 `PWM_AUX_`。不得覆盖任何电机输出。

1. 打开 `Vehicle Setup -> Actuators`，确认所选通道没有分配给 Motor。
2. 把该通道的 Function 设为 `Peripheral via Actuator Set 1`。
3. 对应底层参数是 `PWM_MAIN_FUNC5 = 301`。
4. 在 Actuator Test 中，从中位附近开始小范围移动，逐步找到不堵转的开、合机械端点。
5. 把安全端点写成该通道的 Min/Max。不要未经测试直接长期使用 1000/2000 us。
6. 设置 Disarmed 值为断电前希望保持的安全位置；PX4 是否在未解锁时输出还受 prearm 配置影响。
7. 保存、重启飞控，再回读参数确认。

如果夹爪开合方向相反，可以交换节点的 `open_value` 与 `close_value`，不需要改飞行控制代码。

## 4. 在 Orin 上放置并启动程序

先 source ROS 2 和当前 px4_msgs 工作区：

```bash
source /opt/ros/humble/setup.bash
source ~/px4_ros2_ws/install/setup.bash
python3 ~/robotx/px4_gripper_control.py
```

如果文件实际放在别处，替换最后一行路径。节点看到 `/fmu/out/vehicle_status_v1` 后才接受夹爪命令。

另开一个终端，关闭夹爪：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /gripper/close std_msgs/msg/Bool "{data: true}"
```

打开夹爪：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /gripper/close std_msgs/msg/Bool "{data: false}"
```

检查 FMU 通信：

```bash
ros2 topic hz /fmu/out/vehicle_status_v1
ros2 topic echo --once /fmu/out/vehicle_command_ack
```

## 5. 调整开合量

节点默认把 `-1.0` 当作开、`+1.0` 当作合。真正保护机械端点的是 QGC 中该 PWM 通道经过台架标定的 Min/Max。

如果只想使用端点范围的一部分：

```bash
python3 ~/robotx/px4_gripper_control.py --ros-args \
  -p open_value:=-0.8 \
  -p close_value:=0.7
```

如果输出映射到 `Peripheral via Actuator Set 2`（函数 302），同时启动：

```bash
python3 ~/robotx/px4_gripper_control.py --ros-args \
  -p actuator_number:=2
```

QGC 的 Function 和节点 `actuator_number` 必须一致：301 对应 1，302 对应 2，依次到 306 对应 6。

## 6. 目前无硬件时能验证什么

可以验证：

- Python 语法和 ROS 2 依赖；
- PX4/px4_msgs 版本与消息字段匹配；
- SITL 中节点能收到 `vehicle_status_v1`；
- 命令 187 能获得 `ACCEPTED` ACK。

不能验证：

- 真实输出口是否有 PWM；
- 开合方向及安全机械端点；
- 舵机供电、堵转电流和电磁干扰；
- ACK 后夹爪是否真的动作。PX4 的 ACK 只说明命令被处理。

## 7. 首次硬件测试顺序

1. 拆桨，夹爪不抓取物体，飞控和舵机先不要同时上电。
2. 用万用表确认 BEC 电压和极性。
3. 只用 QGC Actuator Test 从中位小步测试，标定安全 Min/Max。
4. 关闭测试滑块，重启飞控并回读 Function/Min/Max/Disarmed。
5. 启动 Micro XRCE-DDS Agent，确认 `vehicle_status_v1` 持续有数据；本机 SITL 实测约 2 Hz，实机频率可能不同。
6. 启动本节点，先发 Open，再发 Close；随时准备切断舵机电源。
7. 检查飞控是否重启、5 V 电压是否下跌、舵机是否发热或持续嗡鸣。
8. 台架通过后，再决定在哪个任务状态调用 `/gripper/close`；不要一开始就并入自动飞行。

## 8. 常见问题

- 日志显示 `no fresh vehicle_status_v1`：Orin 与 FMU 的 XRCE-DDS 链路还没通，或 PX4/px4_msgs 版本不匹配。
- ACK 是 `ACCEPTED` 但舵机不动：优先检查输出 Function 是否为 301、接错 MAIN/AUX、未进入 prearm/armed 输出状态、信号地未共地或舵机没有独立供电。
- 舵机方向反了：交换 `open_value` 和 `close_value`。
- 舵机撞限位、发热或嗡鸣：立即断舵机电源，缩小 QGC 中该通道的 PWM Min/Max。
- 飞控重启：供电设计不合格，不能继续飞行测试；检查 BEC 容量、共地和电源隔离。

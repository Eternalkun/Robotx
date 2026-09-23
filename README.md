# x500 双指夹爪仿真

在本机 PX4 v1.17 / Gazebo Harmonic / ROS 2 Humble 上实现并验证。
使用现有 x500 的机体、传感器与四个电机，机腹增加 120 g 简化双指夹爪。
蓝色和橙色夹指由同一舵机命令控制，以相反关节轴实现镜像运动。

## 启动与开合

在 WSL Ubuntu-22.04 终端中运行：

```bash
cd /mnt/c/Users/quzhankun/Documents/ChatGPT/robotx/gripper_sim
bash run.sh
```

这条命令启动 Gazebo 窗口、独立 PX4 SITL、Agent 和已有夹爪 ROS 2 节点。
首次等待约 20 秒完成连接；窗口里能看到机腹下的双色夹指。
第二个 WSL 终端运行：

```bash
cd /mnt/c/Users/quzhankun/Documents/ChatGPT/robotx/gripper_sim
bash control.sh open
bash control.sh close
```

两条命令分开执行，就能看到张开和合拢。
控制脚本已设置本示例的 `ROS_DOMAIN_ID=27`；未设置该域的旧终端会控制旧仿真，无法控制这个模型。
不需要再另开默认 8888 Agent 或手工设置 MAIN 5。
在启动终端按 Ctrl+C 关闭本示例，或关闭本示例 Gazebo GUI 窗口。

如果提示 instance 7 已运行或 UDP 8899 被占用，先确认原仿真是否仍在运行。
正常运行时在第二个 Ubuntu 终端执行控制命令，不要再次运行 `run.sh`。
如果原启动终端意外关闭，可用 `ss -lunp 'sport = :8899'` 检查 Agent，
并结合进程命令行确认残留属于本示例后再结束；不要用全局 `pkill` 清除其他仿真。
启动脚本现在处理终端关闭的 SIGHUP；强制杀进程或系统异常仍可能留下独立子进程。

如果想手动发 ROS 命令：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=27 ROS_LOCALHOST_ONLY=0
ros2 topic pub --once /gripper/close std_msgs/msg/Bool '{data: true}'
```

## 实际控制路径

`/gripper/close → px4_gripper_control.py → vehicle_command 187 → PX4 FunctionActuatorSet → SIM_GZ_SV_FUNC1=301 → Gazebo servo_0 → 两个夹指关节`。

原先 `PWM_MAIN_FUNC5=301` 验证的是虚拟 PWM 数值；Gazebo 模型关节使用独立的 `SIM_GZ_SV` 输出组。
没有额外编写一个直接订阅 Bool 然后“播放动画”的控制器，关节目标来自 PX4 原生 Gazebo 舵机输出。
Gazebo 插件使用 [JointPositionController](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1JointPositionController.html)。

启动器在临时目录设置：

| 参数 | 值 | 作用 |
| --- | --- | --- |
| SIM_GZ_SV_FUNC1 | 301 | actuator set 1 映射至 Gazebo 舵机1 |
| SIM_GZ_SV_MIN1 / MAX1 | 0 / 1000 | Gazebo 舵机内部数值范围，不是实机微秒值 |
| SIM_GZ_SV_MINA1 | 35° | `open_value=-1` 对应张开 |
| SIM_GZ_SV_MAXA1 | -8° | `close_value=+1` 对应合拢 |
| SIM_GZ_SV_DIS1 | 500 | 初始/未激活时中间位置 |
| COM_PREARM_MODE | 2 | 本仿真未解锁时允许非电机输出 |

角度上下端的方向是刻意反向配置的。不要把这些仿真参数直接写进真实飞控。
节点启动不发送 Open/Close，但新仿真的输出初始化会使夹指从建模零位移动至中间位置。

## 自动验证与图片

保持启动终端运行，在第二个终端执行：

```bash
bash control.sh verify
```

脚本依次发送打开、关闭、再打开，同时检查：PX4 状态、命令187的 ACK、
Gazebo 舵机目标、左右夹指的真实关节角度和稳定后的角速度。
每个动作保存相机图片；结果在 `results/verification.json` 和 `results/*.png`。
2026-09-07 本机测试：打开两指均约 34.697°，关闭两指均约 -7.927°，再打开通过。

直接观察关节数据：

```bash
GZ_PARTITION=robotx_gripper gz topic -e -n 1 \
  -t /world/gripper_demo/model/x500_gripper_7/joint_state
```

不显示窗口的验证模式为 `bash run.sh --headless`。
相机来自 Gazebo 渲染，用于验证模型实际外观；截图不是生成效果图。

## 隔离与文件

本示例使用 Gazebo partition `robotx_gripper`、ROS domain 27、Agent UDP 8899、
PX4 instance 7（system ID 8）。旧 x500 和旧 Agent 可以继续运行。
每次启动新建 `/tmp/robotx-gripper-*`，里面保存本次日志和参数，不覆盖原 SITL 参数。
Ctrl+C 只关闭本次启动的子进程，日志保留便于排查。不要同时启动两份本示例。
默认读取 `~/robotx/PX4-Autopilot` 和 `~/px4_ros2_ws`；换机器可用环境变量
`PX4_ROOT` 和 `PX4_ROS_WS` 指定路径。

- `models/x500_gripper/model.sdf`：安装座、夹指、碰撞体、质量、关节及其控制器。
- `gripper_world.sdf`：地面、光照、相机和带夹爪 x500。
- `launch.py` / `run.sh`：启动与退出。
- `control.sh` / `verify.py`：手动控制和自动验证。

这是带碰撞体和关节动力学的开合模型。目前没有加入抓取目标物，也未验证夹持后携物飞行、
真实舵机力矩或电流特性；不能将开合通过视为载荷飞行已经验证。

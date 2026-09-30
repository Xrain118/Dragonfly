# Dragonfly 飞行与键盘动臂

源码在 `/home/ubuntu/Dragonfly`。固定演示与自由飞行共用原 Xacro 和尺寸参数。飞行模式移除 world 固定关节，使用六个 Gazebo 电机、六个倾转力矩伺服和两个 effort 轨迹控制器。机械臂反作用来自多刚体动力学，没有额外注入“机械臂扰动力”。

## 构建与环境

本机版本：Ubuntu 22.04、ROS 2 Humble、Gazebo Harmonic 8.15、PX4 v1.16.2、Micro XRCE-DDS Agent v2.4.3，以及 `/home/ubuntu/px4_ros2_ws` 的兼容消息。单独使用 `build-flight` / `install-flight`，原源码改动和构建目录保留。

```bash
cd /home/ubuntu/Dragonfly
bash src/dragonfly_flight/scripts/build.sh
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source /home/ubuntu/gz_ros2_control_ws/install/setup.bash
source /home/ubuntu/px4_ros2_ws/install/setup.bash
source install-flight/setup.bash
```

每个新终端执行 export/source。`PYTHONNOUSERSITE=1` 避免本机用户目录 NumPy 2 与系统 SciPy 冲突。

构建脚本应用 `config/px4_mag_enu.patch` 并编译 PX4。补丁仅在 `PX4_GZ_MAG_ENU=1` 时将 Gazebo 机体 FLU 磁场转换为 PX4 FRD；原生 x500 默认路径保留。Gazebo 使用 ENU 地磁场和 gauss，磁偏角来自 Harmonic 在 Zurich 原点的磁场表。这些 SITL 参数不用于实机。

## 三种启动方式

已完成构建时，可直接在 WSL 终端运行 `bash /home/ubuntu/Dragonfly/src/dragonfly_flight/scripts/sim.sh --gui --record`，另一终端运行同一脚本并传入 `keyboard`。该脚本自动加载下面所需环境。

一次只运行一个入口，固定演示不能与飞行共享控制器。

```bash
# 原固定机体演示（位置接口）
ros2 launch dragonfly_arm gazebo.launch.py

# 理想合力/合力矩调试，不启动 PX4 或旋翼模型
ros2 run dragonfly_flight flight --mode ideal_wrench --scenario hover --gui

# 从地面解锁、平滑起飞，保持悬停并开放键盘
ros2 run dragonfly_flight flight --scenario manual --gui --record
```

目标高度 1.6 m，仿真约半速运行。估计器就绪后才进入 Offboard 并解锁；稳定悬停后显示 `ready=True`。主运行终端 Ctrl+C 结束整次仿真。省略 `--gui` 可无界面运行。

另一 WSL 终端加载环境后：

```bash
export ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=0
ros2 run dragonfly_flight keyboard
```

| 按键 | 动作 |
|---|---|
| L / R | 选择左 / 右臂，不产生运动 |
| 1～5 | 底座、肩、肘、腕俯仰、末端滚转 |
| + 或 = / - | 所选关节点动 ±2° |
| Space | 两臂减速停止并保持，飞行继续 |
| H | 所选臂返回 `[0,0,π/2,0,0]` |
| P | home → 肩 +5° → 停留 2 s → home |
| Esc / Ctrl+C | 停止机械臂并退出键盘，飞行继续 |

按键由获得焦点的 WSL 终端接收。正方向遵循 URDF 关节轴。显示实际角、目标角和状态。一次只执行一只臂的一个动作，动作中运动键丢弃，停止键优先。速度 0.10 rad/s，加速度 0.20 rad/s²。心跳丢失 0.5 墙钟秒、关节状态过期或失去稳定悬停条件时，手动动作停止。

活动范围为 home 附近 ±10°，位于原 URDF 限位内。部分组合会接近起落架，例如左底座负向、右底座正向约 5°。管理器用加 3 mm 余量的碰撞几何包围盒检查路径，拒绝接近起落架的动作或回 home 路径，不绕障规划；这不是全工作空间避碰保证。

左右 `follow_joint_trajectory` action 及关节顺序保留。管理器通过同一轨迹控制器的 `joint_trajectory` topic 下发平滑轨迹。飞行入口不启动旧 demo 和 `tilt_controller`；不要另开客户端直接发送轨迹绕过管理器。

## 自动验收与物理诊断

```bash
# 每轮：30秒固定臂悬停、四段平移、左右臂标准回放各一次
ros2 run dragonfly_flight flight --scenario acceptance --record --output log/my_acceptance_1
ros2 run dragonfly_flight verify log/my_acceptance_1
# 使用新目录重复三轮；verify 未通过返回非零退出码。
# 或一次执行三轮并生成曲线：
bash src/dragonfly_flight/test/run_acceptance.sh

# 先稳定悬停，再短时自由落体 / 单轴已知合力
ros2 run dragonfly_flight flight --mode ideal_wrench --scenario freefall --output log/my_freefall
ros2 run dragonfly_flight flight --mode ideal_wrench --scenario force_x --output log/my_force_x
# force_y、force_z 同理，结果为 diagnostic.json。

# 无重力、无旋翼、无理想外力，检查内部动臂和机体反向响应
ros2 run dragonfly_flight flight --mode physics --scenario physics --step .002 --output log/my_momentum
# .001、.0005 步长使用新目录重复。

colcon test --build-base build-flight --install-base install-flight --packages-select dragonfly_arm dragonfly_flight
colcon test-result --test-result-base build-flight --verbose
```

Gazebo 真值验收：30 s 位置 RMS ≤0.10 m、姿态 RMS ≤3°、饱和不持续超过 1 s；平移完成 5 s 后误差 ≤0.10 m；动臂峰值 ≤0.25 m / 8°、动作结束 5 s 后 ≤0.10 m / 3°。实际肩关节行程必须达到 4.5°，避免把未执行动作算作通过。理想力成绩不能用于旋翼悬停验收。

输出目录保存模型、控制器、PX4 参数快照、`flight.csv`、`events.json` 和进程日志；`--record` 保存 ROS bag，PX4 模式保存 ULog。`verify` 写 `metrics.json`。物理诊断保存 `momentum.csv`：force 列为线动量 kg·m/s，torque 列为绕世界原点角动量 kg·m²/s。

## 模型与控制边界

Gazebo 传感器 → PX4 估计 → ROS 位置/几何姿态 PD 与有界分配 → PX4 direct_actuator 输出 → Gazebo。飞行循环 100 Hz、仿真时间；起飞前用真值对齐世界原点，起飞后控制反馈来自 PX4。真值用于评价、机体 TF 和动臂稳定条件。

总质量 4.628 kg，home 质心约 `(0.02649,0,-0.08799)` m，全机惯量由 URDF 合成。力矩统一以机体原点为参考。分配包含共同倾转轴、变化的旋翼位置和反扭矩，每周期最多三次有界最小二乘，均衡推力项抑制冗余解漂移。残差是目标倾转角下的非线性残差，实际电机/倾转滞后另记录。

仿真假设：推重比 2、电机时间常数 0.05 s、反扭矩/推力比 0.02 m、倾转 ±π/2 / 0.5 rad/s / 8 N·m。两臂由 500 Hz effort 轨迹控制器输出力矩，执行前限幅 ±5 N·m。初始化一次 home，正常运行不重设关节位置。参数集中于 `config/flight.yaml`、原 `model_dimensions.yaml` 和生成的控制器配置，全部为可替换估值。

飞行中只下发关节力矩。Gazebo 物理系统在存在 `JointForceCmd` 时优先执行力矩，未启用位置或速度伺服作为辅助。

`/dragonfly/truth` 位姿与线/角速度均为 world ENU。`/dragonfly/efforts`、`/dragonfly/rotor_speeds` 用 CSV 字符串兼容当前 bridge，顺序为左5/右5关节和 A左右/B左右/C左右电机。`/dragonfly/target` 仅在 manual 接受 world 位置目标，姿态始终保持 ENU 零航向水平。

PX4 状态过期 0.2 仿真秒、非法状态、连续分配失败或离开已解锁 Offboard 将中止。监控先停止物理进程，再关闭本次子进程并保存日志，也监控控制进程卡住。Offboard 丢失设为终止，不依赖 x500 自动降落。直接执行器模式绕过原生着陆检测器的推力参考，因此关闭 SITL 自动着陆/起飞超时解锁计时，退出由本入口负责。

本轮没有风场、实机标定、末端控制、抓取、动臂前馈或扰动观测器。同模型内部接触沿用原模型配置；几何间隙检查不能证明未建模接触的实物安全性。仿真结果不代表实物抗扰性能。

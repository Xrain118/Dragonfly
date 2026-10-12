# Dragonfly 全驱动倾转六旋翼与双五轴机械臂

本工程面向搭载双机械臂的全驱动倾转六旋翼无人机，目标是实现双臂主从遥操作，并研究机械臂运动与飞行器之间的动力学耦合及控制。平台由三组双电机、双轴倾转动力单元组成，共六路旋翼推力和六个倾转关节；机腹倒置安装两套 RoArm-M3 五轴机械臂。

**截至 2026-10-12，工程已完成模型、原生 PX4 控制集成与无风 SITL 验证，尚未完成实机双臂主从遥操作或整机实飞验收。** Gazebo 中的夹爪保持闭合，开合执行器不计入每臂五个运动关节。

## 当前进度

| 子系统 | 已完成 | 当前边界 |
|---|---|---|
| 整机与双臂模型 | 共用 Xacro/URDF，图纸尺寸、安装外参、质量汇总和逐关节峰值限幅 | 分段质量、质心、惯量及部分外形仍为估算 |
| 双臂运动学 | FK、解析 IK、雅可比、工具位姿发布及左右 IK 服务 | 求解不自动下发动作；不保证全工作空间无碰撞 |
| 飞行控制 | 原生 PX4 位置/速度与姿态/角速度控制，六电机六倾转非线性有界分配，独立机型 40160 | 适配 PX4 v1.16.2 的 SITL；硬件输出与执行器尚待标定 |
| 仿真动臂 | effort 力矩伺服、折叠起飞后同步展开、±5° 键盘点动、限速、心跳停止和起落架间隙检查 | 当前手动动作一次只执行一只臂；不是双臂实机主从闭环 |
| 监控与验收 | PlotJuggler 面板、CSV/ROS bag/ULog 记录、悬停/平移/左右动臂自动验收 | 最新成绩为 2026-10-09 的三轮无风仿真 |
| 机械臂固件 | 已确认 ESP32/Arduino RoArm-M3 整臂控制源码及通信入口 | 固件位于仓库外；ROS 硬件对接、烧录版本确认和实机验收未完成 |

当前没有风场、抓取或接触作业，也没有专用动臂前馈、扰动观测器或学习补偿。模型与飞行验收结果不代表实物抗扰性能。

## 系统架构

默认飞行链路为：

```text
Gazebo 传感器 → PX4 EKF → PX4 位置/速度控制 → PX4 姿态/角速度控制
             → PX4 非线性有界控制分配 → 六电机与六倾转伺服 → Gazebo
ROS 2 → Offboard 位置轨迹与机械臂轨迹
机械臂轨迹 → effort 控制器 → 关节力矩 → 整机多刚体动力学
```

默认 `px4_native` 模式由 PX4 输出飞行执行器指令，ROS 不发布电机/倾转指令。机械臂对机体的反作用由 Gazebo 多刚体动力学产生。控制使用 PX4 估计状态；Gazebo 真值用于起飞前原点对齐、评价、机体 TF 和动臂许可。实机接入时需要替换依赖真值的许可与对齐逻辑。

整机模型质量为 **5.415 kg**，两臂各 **0.9735 kg**。原生控制使用固定的竖直构型标称质心与惯量，起飞、展开及动臂期间不随关节实时更新。旧 `px4_actuators` 模式保留 ROS 控制/分配调试路径；`ideal_wrench` 和 `physics` 用于物理诊断，不能作为旋翼飞行验收成绩。

| 目录/说明 | 内容 |
|---|---|
| [模型、参数与接口](src/dragonfly_arm/docs/model.md) | 坐标、几何、图纸/厂家参数与估算值、质量及力矩限幅、ROS 接口 |
| [双臂运动学](src/dragonfly_arm/docs/kinematics.md) | FK/IK/雅可比、Python API、ROS 服务和舵机角度边界 |
| [飞行与键盘动臂](src/dragonfly_flight/README.md) | 环境、启动、按键、监控、异常处置及验收命令 |
| [原生 PX4 适配](src/dragonfly_flight/config/px4_vector/README.md) | 补丁、参数、通道映射、控制链和 C++ 回归入口 |
| [历史验证记录](src/dragonfly_flight/docs/verification.md) | 2026-09-30 的旧 direct-actuator 结果及其限制 |

### 当前机械臂固件与姿态

用户于 2026-10-12 确认，当前固件源代码位于本机 `C:\Users\LENOVO\Desktop\arm`：工程为 `RoArm-M3_example`，依赖库为 `src`。主程序为 `RoArm-M3_example.ino`，整臂控制与配置分别在 `RoArm-M3_module.h`、`RoArm-M3_config.h`。固件包含 UART JSON、Wi-Fi/HTTP 和 ESP-NOW 入口。该路径是本机外部源码位置，不是克隆本仓库后自动获得的内容；本次未导入或修改固件。

| 姿态 | 关节角（rad） | 用途 |
|---|---|---|
| 仿真折叠姿态 | `[0, 0, π/2, 0, 0]` | PX4 仿真在地面出生与起飞 |
| 仿真竖直姿态 | `[0, 0, 0, 0, 0]` | 展开后的工作姿态，键盘 H/P 的基准 |
| 当前固件自定义上电姿态 | `[0.124252444, -1.526310884, 2.997398460, -0.150330117, 0.006135923]`，夹爪另为 `3.129320807` | `moveToRecordedInitialPose()` 使用速度 100、加速度 8 发送目标，函数内部不等待精确到位 |

五轴顺序均为底座、肩、肘、腕俯仰、末端滚转；夹爪独立。上述固件状态来自源码检查，不等于已确认实物烧录版本。图纸模型采用末端偏移 13.29 mm，当前固件配置仍为 13.69 mm，硬件对接时需统一这一 0.40 mm 差异。

## 快速运行

实际运行仓库为 WSL Ubuntu 的 `/home/ubuntu/Dragonfly`，远端为 `Xrain118/Dragonfly`；Windows 的 `Documents/ChatGPT/dragonfly` 保存本机输出，桌面镜像不作为运行源码入口。

本机环境为 Ubuntu 22.04、ROS 2 Humble、Gazebo Harmonic 8.15、PX4 v1.16.2 和 Micro XRCE-DDS Agent v2.4.3；还需要匹配的 `px4_msgs` 与 `gz_ros2_control` 工作区。构建依赖与版本检查见 [运行说明](src/dragonfly_flight/README.md#构建与环境)。

在 WSL 终端执行：

```bash
cd /home/ubuntu/Dragonfly
bash src/dragonfly_flight/scripts/build.sh
bash src/dragonfly_flight/scripts/sim.sh --gui --record
```

构建使用独立 `build-flight/install-flight`。等待起飞、双臂展开和机体再次稳定，终端显示 `ready=True` 后，在两个新终端分别运行：

```bash
bash /home/ubuntu/Dragonfly/src/dragonfly_flight/scripts/sim.sh keyboard
bash /home/ubuntu/Dragonfly/src/dragonfly_flight/scripts/sim.sh monitor
```

键盘 `L/R` 选臂，`1～5` 选关节，`+/-` 点动 5°，Space 停止两臂并保持，H 回到竖直姿态，P 回放肩关节 +5° 动作。键盘终端需获得焦点；完整限位、心跳和退出行为见运行说明。一次只运行一套仿真。

固定机体演示使用同一模型，机体固定在 z=1.60 m。完成上述构建后，在另一空闲终端执行：

```bash
cd /home/ubuntu/Dragonfly
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source /home/ubuntu/gz_ros2_control_ws/install/setup.bash
source install-flight/setup.bash
ros2 launch dragonfly_arm demo.launch.py
```

只查看模型使用 `gazebo.launch.py`；固定演示使用位置接口，自由飞行动臂使用力矩接口。

## 验证结果

最新飞行验收日期为 **2026-10-09**，三轮均采用当前 5.415 kg 模型和独立机型 40160。每轮包含折叠起飞、双臂展开、30 s 固定臂悬停、四段平移和左右肩关节各一次 +5° 回放。

| 轮次 | 悬停位置 RMS | 悬停姿态 RMS | 平移后 5 s 最大误差 | 动臂后 5 s 最大误差 | 左/右肩实际行程 | 结论 |
|---|---:|---:|---:|---:|---:|---|
| 1 | 5.42 cm | 1.04° | 6.59 cm | 5.58 cm | 4.83° / 4.85° | 通过 |
| 2 | 4.22 cm | 0.52° | 5.47 cm | 1.49 cm | 4.83° / 4.84° | 通过 |
| 3 | 4.83 cm | 0.57° | 4.50 cm | 3.69 cm | 4.84° / 4.85° | 通过 |

验收阈值：悬停位置 RMS ≤0.10 m、姿态 RMS ≤3°、连续饱和 ≤1 s；平移完成 5 s 后误差 ≤0.10 m；动臂峰值 ≤0.25 m/8°，动作完成 5 s 后误差 ≤0.10 m/3°，实际肩关节行程 ≥4.5°。三轮连续饱和均为 0 s，十个机械臂关节实际力矩均未超限，采样起落架间隙检查通过；这不构成全工作空间碰撞认证。

记录保存在本机 `log/drawing_native_acceptance_20261009_1`、`_2`、`_3`，包括模型、参数、控制链证明、CSV、曲线、ROS bag 和 ULog。日志不纳入仓库；2026-10-05 的旧 4.628 kg 模型成绩及 2026-09-30 的旧调试模式成绩另作历史记录，不混作当前结果。

2026-10-12 重新构建 PX4 与 ROS 两包，ROS 回归汇总 69 项（59 个 pytest 用例、10 个 CTest 入口）、9 项原生 PX4 C++ 检查及 16 项上游位置控制检查全部通过。重新生成的模型、PX4 参数和适配源码哈希与验收依据一致，三轮历史数据按当前规则复核通过；本次没有重新飞行三轮。2026-10-09 固定机体 Gazebo 冒烟测试也已通过，包含四个控制器、16 个关节状态、两套工具位姿及 IK 服务。

复验命令：

```bash
cd /home/ubuntu/Dragonfly
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source /home/ubuntu/gz_ros2_control_ws/install/setup.bash
source /home/ubuntu/px4_ros2_ws/install/setup.bash
source install-flight/setup.bash
colcon test --build-base build-flight --install-base install-flight \
  --packages-select dragonfly_arm dragonfly_flight --event-handlers console_direct+
colcon test-result --test-result-base build-flight --verbose
# 需要重新执行完整三轮飞行验收时：
bash src/dragonfly_flight/test/run_acceptance.sh log/my_native_acceptance
```

## 下一阶段

1. **硬件基线与标定**：确认各臂设备、舵机 ID/零位/方向/真实端点、肩部双舵机协同、安装外参、供电与固件版本；统一模型和固件参数。
2. **主从映射与通信**：确定地面主臂、机载从臂及上位机分工，完成 ROS 硬件驱动、真实反馈、接管对齐、限速及失联恢复。
3. **固定机体双臂实机闭环**：完成单关节、单臂、双臂连续输入、时延、掉线和长期运行验收。
4. **实机飞控适配**：标定六电机与六倾转输出、推力、质量/质心/惯量和定位状态，替换依赖 Gazebo 真值的对齐与动臂许可；可与机械臂联调并行推进。
5. **整机地面联调与逐级飞行**：先合并真实链路及故障场景，再按固定臂悬停、预设展开、单臂、双臂、实时遥操作逐级验证；抓取和接触作业另行验收。

## 许可证

Copyright 2026 Xrain118。Apache License 2.0，详见 [LICENSE](LICENSE)。

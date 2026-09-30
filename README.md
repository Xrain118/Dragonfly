# Dragonfly 矢量六旋翼与双五轴机械臂

本工程使用 ROS 2 Humble、Gazebo Harmonic，按实物照片建立三组双电机、双轴倾转动力单元，以及机腹倒置安装的两套相同五轴机械臂。夹爪作为闭合的固定几何体；其开合执行器不算入机械臂五个运动关节。

原演示仍将机体固定在 z=1.60 m。新增 `dragonfly_flight` 从同一 Xacro 生成自由机体，通过 PX4 SITL、六路旋翼推力和三组双轴倾转飞行，机械臂使用物理力矩伺服。启动、键盘操作和自动验收见 [飞行与动臂说明](src/dragonfly_flight/README.md)。**质量、惯量和执行器仍为仿真估值，结果不代表实物抗扰性能；未接入 ESP32 硬件。**

## 尺寸和坐标

所有长度为米、角度为弧度。集中参数文件：

`src/dragonfly_arm/config/model_dimensions.yaml`

- `measured`：用户提供的机架、动力单元和双臂安装间距。
- `arm_kinematics`：RoArm-M3 源码当前启用的尺寸、初始姿态和运动关节软件限位，供 Python 与 URDF 共用。
- `estimated`：照片估计与第一版设计假设。
- `simulation`：展示用质量、限位、速度与力矩占位值，并非实物标定。

机体原点为三角形几何中心；+X 指向 A 单元、+Y 向左、+Z 向上。动力单元安装中心的平面坐标为：

| 单元 | X | Y | Z |
|---|---:|---:|---:|
| A | 0.692820 | 0 | 0.06 |
| B | -0.346410 | 0.600000 | 0.06 |
| C | -0.346410 | -0.600000 | 0.06 |

三角形边长为 1.2 m。安装中心高度 0.06 m 为估计值。每组电机沿局部 Y 轴对称设置，轴心距 0.36 m；电机半径暂取 0.015 m、高 0.020 m。每个桨有三片叶片，桨半径 0.116 m；碰撞模型采用同半径的薄圆盘。

倾转运动采用用户接受的近似：
`Rz(单元方位角) × Rx(outer) × Ry(inner)`。两轴相交，outer 为径向，inner 在零位时为切向并随 outer 转动。零位所有桨盘水平；每组双电机随同一支架倾转。旋翼 link 的 +Z 为正推力方向，旋翼 link 的原点在桨毂中心。轴序、轴偏移、硬件零点和真实限位后续均需校准。

两套机械臂底座均位于机体中心下方 0.05 m，分别安装在 `y=+0.15 m`（左臂）和 `y=-0.15 m`（右臂），中心距严格为 0.30 m。中心控制板下方设置一块 `0.14 × 0.40 × 0.012 m` 的连续承载板，覆盖两套基座的完整投影；两根短适配器从承载板底面连续连接至机械臂基座。两臂同向平行安装，均绕 X 轴旋转 π，机械臂基座局部 +Z 朝机腹下方。轴序为：
`底座回转 → 肩俯仰 → 肘俯仰 → 腕俯仰 → 末端轴向旋转`。
关节轴在各自局部坐标系中为 Z、Y、Y、Y、Z；第五关节坐标系含末端安装角，其 +Z 沿末端偏置段方向。

| 区段 | 源码位移向量（m） |
|---|---|
| 基座至肩 | `(0, 0, 0.12606)` |
| 肩至肘，在肩转动后的局部系 | `(0.03000, 0, 0.23682)` |
| 肘至腕，在肘转动后的局部系 | `(0, 0, 0.14449)` |
| 腕至固定工具点，在腕转动后的局部系 | `(0.01369, 0, 0.17167)` |

旧的 0.40 m 总长近似已替换。固件初始姿态为 `[0, 0, π/2, 0, 0]`，相应工具点在各自基座坐标系中的位置为 `(0.34616, 0, 0.34919) m`。全零关节角与固件初始姿态不同。

`left_tool0`、`right_tool0` 是源码固定工具参考点，夹爪开合不改变它。各臂关节限位依次为 `±π、±π/2、[0,π]、±π/2、±π`；倾转限位仍为 ±π/2。机械臂限位表示源码控制函数的理想弧度范围，真实舵机端点和零位需标定；肘部原始指令上限 3071 ticks 比理想 π 少一个编码器步。完整坐标定义、公式和实物参考边界见 [RoArm-M3 运动学说明](src/dragonfly_arm/docs/kinematics.md)。

## 工程组织与接口

- `urdf/dragonfly_arm.urdf.xacro`：整机入口及 ros2_control。
- `urdf/vector_platform.xacro`：三角机架、起落架、倾转单元与三叶桨。
- `urdf/five_axis_arm.xacro`：五轴机械臂与固定夹爪。
- `urdf/geometry_macros.xacro`：几何、惯量和关节接口宏。
- `worlds/model_preview.world`：内置地面和灯光的展示场景，避免启动时依赖在线模型下载。

原四旋翼文件已从有效源码移出，改造前副本保存在工作区 `log/model_before_vector_20260923/dragonfly_quadrotor.urdf.xacro`，不会安装为新模型。

| 接口 | 内容 |
|---|---|
| `/left_arm_controller/follow_joint_trajectory` | `left_joint_1` 至 `left_joint_5`，按此顺序 |
| `/right_arm_controller/follow_joint_trajectory` | `right_joint_1` 至 `right_joint_5`，按此顺序 |
| `/tilt_controller/follow_joint_trajectory` | A outer/inner、B outer/inner、C outer/inner |
| `/joint_states` | 16 个受控关节的位置、速度状态 |
| `/left_arm/tool_pose`、`/right_arm/tool_pose` | `PoseStamped`，分别以 `left_base_link`、`right_base_link` 为参考系 |
| `/left_arm/solve_ik`、`/right_arm/solve_ik` | `dragonfly_arm/srv/SolveIK`，仅返回逆解，不发送运动命令 |
| `/tf`、`/tf_static`、`/robot_description` | 全机坐标与模型 |

倾转关节完整名称为 `unit_a_outer_joint`、`unit_a_inner_joint`，B/C 同理。
原 `/arm_controller/...` 接口不再保留。旧单臂或六轴命令必须改为左右两个五关节接口；不存在 `joint_6`，夹爪不参与运动控制。

## 构建与启动

在 WSL Ubuntu 终端执行，Windows 编辑器可以通过 WSL 远程连接打开同一工程。不要在 Windows 目录另建一份不同步的模型。

```bash
cd /home/ubuntu/Dragonfly
source /opt/ros/humble/setup.bash
source /home/ubuntu/gz_ros2_control_ws/install/setup.bash
colcon build --symlink-install --packages-select dragonfly_arm
source install/setup.bash
ros2 launch dragonfly_arm gazebo.launch.py
```

Gazebo Harmonic 的 `gz_ros2_control` 已按 Humble 分支安装到 `/home/ubuntu/gz_ros2_control_ws`。当前工程的 `install/setup.bash` 会链式加载该工作区，因此完成一次上述构建后，新终端日常启动只需 source ROS 2 和本工程：

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/Dragonfly/install/setup.bash
ros2 launch dragonfly_arm gazebo.launch.py
```

演示等待控制器 active 后，先将双臂移到固件初始姿态，再运行小幅独立轨迹并返回初始姿态；随后运行六个倾转关节并回零：

```bash
ros2 launch dragonfly_arm demo.launch.py
```

无图形模式附加 `gui:=false`；只查看暂停模型使用
`ros2 launch dragonfly_arm gazebo.launch.py paused:=true`。
暂停时演示轨迹不会前进。不要同时启动两个使用同一 ROS/Gazebo 地址的实例。

Gazebo 启动文件同时启动运动学节点。仅使用计算服务时可执行 `ros2 run dragonfly_arm kinematics_node`；发布工具位姿还需要外部 `/joint_states`。Python API、服务请求示例和状态码见运动学说明。

如果图形驱动异常，可在当前 WSL 终端设置
`export LIBGL_ALWAYS_SOFTWARE=1` 后重启图形仿真。

## 验证

```bash
xacro src/dragonfly_arm/urdf/dragonfly_arm.urdf.xacro > /tmp/dragonfly_vector.urdf
check_urdf /tmp/dragonfly_vector.urdf
colcon test --packages-select dragonfly_arm --event-handlers console_direct+
colcon test-result --verbose
bash src/dragonfly_arm/test/smoke_test.sh
```

测试涵盖源码初始位姿、全关节范围随机 FK→IK→FK 回算、多个构型与 ±π 等价角、基座/肘部奇异点、限位与不可达状态、雅可比有限差分、随机 URDF 对照及 ROS 输入校验；同时保留机架几何、16 个受控关节、倾转轴与惯量检查。冒烟测试启动 Gazebo Harmonic，检查四个控制器 active、16 个有限关节状态、两个工具位姿及两个 IK 服务，并等待双臂和倾转轨迹成功完成。可给脚本附加图片输出目录，在验证后拍摄三张真实 Gazebo 场景图。

新模型仍使用简化碰撞体。小幅演示可用于检查模型运动；全部关节极限组合没有做无碰撞认证。增加真实飞行和风扰之前，需要标定质量/质心/惯量、推力参数、舵机轴线和控制时延，并建立可自由运动的飞行模型。

仿真运行且未暂停时，可在另一 WSL 终端获取 Gazebo 场景渲染图：

```bash
source /opt/ros/humble/setup.bash
cd /home/ubuntu/Dragonfly
python3 src/dragonfly_arm/test/capture_gazebo.py log/model_validation
```

该工具需要 `ros_gz_image` 与 Pillow，只临时添加无碰撞的观察相机，截图后自动删除相机，不改变机器人关节。输出为整机斜视、侧视、双机械臂近景，可在 WSLg 窗口截图不正常时使用；它不依赖当前环境中与 NumPy 2 不兼容的 OpenCV 二进制包。

## 许可证

Copyright 2026 Xrain118。Apache License 2.0，详见 LICENSE。

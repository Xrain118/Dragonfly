# Dragonfly 矢量六旋翼与五轴机械臂

本工程使用 ROS 2 Humble、Gazebo Classic 11，按实物照片建立三组双电机、双轴倾转动力单元，以及机腹倒置安装的五轴机械臂。夹爪作为闭合的固定几何体；其开合执行器不算入机械臂五个运动关节。

这是固定机体的外形与运动学近似模型。机体通过固定关节悬置在 z=1.60 m。**尚无飞行推力、旋翼转速、风场、真实惯量标定或 ESP32 硬件驱动，不可把演示结果作为抗扰性能验证。**

## 尺寸和坐标

所有长度为米、角度为弧度。集中参数文件：

`src/dragonfly_arm/config/model_dimensions.yaml`

- `measured`：用户提供尺寸，其中机械臂总长 0.40 m 是约数。
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

机械臂底座安装在机体中心下方 0.05 m，绕 X 轴旋转 π，伸直零位朝下。轴序为：
`底座回转 → 肩俯仰 → 肘俯仰 → 腕俯仰 → 末端轴向旋转`。
关节轴在各自局部坐标系中为 Z、Y、Y、Y、Z。

| 区段 | 估算长度 |
|---|---:|
| 安装面至肩关节（含底座回转） | 0.045 |
| 肩至肘 | 0.150 |
| 肘至腕俯仰 | 0.085 |
| 腕俯仰至末端旋转 | 0.035 |
| 末端旋转至夹爪尖端 | 0.085 |
| 伸直总长 | 0.400 |

底座回转轴距安装面 0.020 m。`tool0` 位于夹爪尖端。五轴机械臂和六个倾转关节仿真限位暂取 ±π/2，不代表真实机械限位。

## 工程组织与接口

- `urdf/dragonfly_arm.urdf.xacro`：整机入口及 ros2_control。
- `urdf/vector_platform.xacro`：三角机架、起落架、倾转单元与三叶桨。
- `urdf/five_axis_arm.xacro`：五轴机械臂与固定夹爪。
- `urdf/geometry_macros.xacro`：几何、惯量和关节接口宏。
- `worlds/model_preview.world`：内置地面和灯光的展示场景，避免启动时依赖在线模型下载。

原四旋翼文件已从有效源码移出，改造前副本保存在工作区 `log/model_before_vector_20260923/dragonfly_quadrotor.urdf.xacro`，不会安装为新模型。

| 接口 | 内容 |
|---|---|
| `/arm_controller/follow_joint_trajectory` | `joint_1` 至 `joint_5`，按此顺序 |
| `/tilt_controller/follow_joint_trajectory` | A outer/inner、B outer/inner、C outer/inner |
| `/joint_states` | 11 个受控关节的位置、速度状态 |
| `/tf`、`/tf_static`、`/robot_description` | 全机坐标与模型 |

倾转关节完整名称为 `unit_a_outer_joint`、`unit_a_inner_joint`，B/C 同理。
旧六轴命令需要改成五个关节；`joint_6` 已移除，不能用它控制夹爪。

## 构建与启动

在 WSL Ubuntu 终端执行，Windows 编辑器可以通过 WSL 远程连接打开同一工程。不要在 Windows 目录另建一份不同步的模型。

```bash
cd /home/ubuntu/Dragonfly
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select dragonfly_arm
source install/setup.bash
ros2 launch dragonfly_arm gazebo.launch.py
```

演示会依次运行五轴机械臂、单轴倾转、组合倾转，再回到零位：

```bash
ros2 launch dragonfly_arm demo.launch.py
```

无图形模式附加 `gui:=false`；只查看暂停模型使用
`ros2 launch dragonfly_arm gazebo.launch.py paused:=true`。
暂停时演示轨迹不会前进。不要同时启动两个使用同一 ROS/Gazebo 地址的实例。

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

测试涵盖 11 个关节的控制契约、坐标树、五轴正运动学、三个安装中心间距、双电机间距、桨尺寸、机械臂长度、单轴/组合倾转推力方向，以及质量惯量的有效性。冒烟测试启动 Gazebo，检查三个控制器和关节状态，并验证两组轨迹均成功完成。

新模型仍使用简化碰撞体。小幅演示可用于检查模型运动；全部关节极限组合没有做无碰撞认证。增加真实飞行和风扰之前，需要标定质量/质心/惯量、推力参数、舵机轴线和控制时延，并建立可自由运动的飞行模型。

仿真运行且未暂停时，可在另一 WSL 终端获取 Gazebo 场景渲染图：

```bash
source /opt/ros/humble/setup.bash
cd /home/ubuntu/Dragonfly
python3 src/dragonfly_arm/test/capture_gazebo.py log/model_validation
```

该工具需要 cv_bridge 与 OpenCV，只临时添加无碰撞的观察相机，截图后自动删除相机，不改变机器人关节。输出为整机斜视、侧视、机械臂近景，可在 WSLg 窗口截图不正常时使用。

## 许可证

Copyright 2026 Xrain118。Apache License 2.0，详见 LICENSE。

# Dragonfly 六轴机械臂仿真

这是一个面向 ROS 2 Humble 与 Gazebo Classic 11 的最小六轴机械臂仿真工程。机械臂使用 Xacro 基本几何体构建，通过 `gazebo_ros2_control` 和 `JointTrajectoryController` 驱动。

## 已实现功能

- 固定底座的六旋转关节机械臂，关节名为 `joint_1` 到 `joint_6`
- 完整的视觉、碰撞、质量和惯量参数
- `/joint_states`、`/tf`、`/tf_static` 和 `/robot_description`
- `/arm_controller/follow_joint_trajectory` 轨迹 action
- 一键 Gazebo 启动和四点往返轨迹演示

## 环境检查与依赖

所有命令都应在 WSL Ubuntu 终端中执行。不要在 Windows PowerShell 中通过 `\\wsl$` 路径运行 `colcon`。

```bash
cd /home/ubuntu/Dragonfly
source /opt/ros/humble/setup.bash

ros2 --help
gazebo --version
```

如需重新安装依赖：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  python3-colcon-common-extensions \
  python3-rosdep

rosdep install --from-paths src --ignore-src -r -y
```

## 构建

```bash
cd /home/ubuntu/Dragonfly
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

每次新开终端后，需要重新执行：

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/Dragonfly/install/setup.bash
```

## 启动

只启动 Gazebo、机械臂和控制器：

```bash
ros2 launch dragonfly_arm gazebo.launch.py
```

启动并自动执行一次约 15 秒的往返轨迹：

```bash
ros2 launch dragonfly_arm demo.launch.py
```

无图形界面运行：

```bash
ros2 launch dragonfly_arm demo.launch.py gui:=false
```

以暂停状态启动模型检查（暂停时不会执行轨迹）：

```bash
ros2 launch dragonfly_arm gazebo.launch.py paused:=true
```

只启动仿真后，也可以在另一个已加载工作空间环境的终端手动运行演示：

```bash
ros2 run dragonfly_arm demo_trajectory
```

## 状态检查

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 action list | grep follow_joint_trajectory
ros2 node list
```

正常状态下应看到：

```text
joint_state_broadcaster  joint_state_broadcaster/JointStateBroadcaster  active
arm_controller           joint_trajectory_controller/JointTrajectoryController  active
```

模型和测试检查：

```bash
xacro src/dragonfly_arm/urdf/dragonfly_arm.urdf.xacro > /tmp/dragonfly_arm.urdf
check_urdf /tmp/dragonfly_arm.urdf
colcon test --packages-select dragonfly_arm --event-handlers console_direct+
colcon test-result --verbose
```

完整的无界面 Gazebo 冒烟测试会启动仿真、检查控制器和 `/joint_states`，并等待演示轨迹完成：

```bash
bash src/dragonfly_arm/test/smoke_test.sh
```

## WSLg 图形故障排查

先确认 WSLg 环境变量存在：

```bash
echo "$DISPLAY"
echo "$WAYLAND_DISPLAY"
```

如果 Gazebo 启动后黑屏、闪退或 OpenGL 报错，可以尝试软件渲染：

```bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch dragonfly_arm gazebo.launch.py
```

如果只需要验证控制链，使用 `gui:=false` 绕过图形客户端。关闭异常残留的 Gazebo 进程后再重新启动：

```bash
pkill -f gzserver || true
pkill -f gzclient || true
```

## 当前边界

该版本用于跑通 ROS 2 仿真控制链，不包含夹爪、MoveIt、逆运动学、CAD 网格或真实硬件驱动。后续可以在保持关节命名不变的情况下替换外观模型、增加末端执行器并接入 MoveIt 2。

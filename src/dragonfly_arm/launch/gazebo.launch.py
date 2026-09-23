"""Launch the fixed-base vector hexarotor and five-axis arm in Gazebo Classic."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    paused = LaunchConfiguration("paused")
    use_sim_time = LaunchConfiguration("use_sim_time")

    package_share = FindPackageShare("dragonfly_arm")
    xacro_file = PathJoinSubstitution([package_share, "urdf", "dragonfly_arm.urdf.xacro"])
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " use_sim_time:=",
                use_sim_time,
            ]
        ),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments={
            "gui": gui,
            "pause": paused,
            "world": PathJoinSubstitution([package_share, "worlds", "model_preview.world"]),
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": use_sim_time}
        ],
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_dragonfly_arm",
        output="screen",
        arguments=[
            "-entity",
            "dragonfly_arm",
            "-topic",
            "robot_description",
            "-timeout",
            "120",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )

    tilt_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tilt_controller", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "60"],
        output="screen",
    )

    start_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    start_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, tilt_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start the Gazebo graphical client.",
            ),
            DeclareLaunchArgument(
                "paused",
                default_value="false",
                description="Start Gazebo with physics paused.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo simulation clock.",
            ),
            gazebo,
            robot_state_publisher,
            spawn_robot,
            start_joint_state_broadcaster,
            start_arm_controller,
        ]
    )

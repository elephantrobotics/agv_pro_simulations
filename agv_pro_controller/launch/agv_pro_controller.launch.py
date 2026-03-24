#!/usr/bin/env python3
"""
AGV Pro Controller Launch File
仅负责启动控制器相关节点
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
    EmitEvent,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessIO
from launch.events import Shutdown
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ========== 1. 定义启动参数 ==========
    namespace = LaunchConfiguration("namespace", default="")
    use_sim = LaunchConfiguration("use_sim", default="False")

    # ========== 2. 声明启动参数 ==========
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Add namespace to all launched nodes.",
    )

    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether to use simulation.",
        choices=["True", "False"],
    )

    # ========== 3. 控制器配置 ==========
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("agv_pro_controller"),
            "config",
            "agv_control.yaml",
        ]
    )

    # ========== 4. 启动 controller_manager (仅硬件模式) ==========
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("mecanum_drive_controller/reference_unstamped", "cmd_vel"),
            ("mecanum_drive_controller/odometry", "odom"),
            (
                "mecanum_drive_controller/transition_event",
                "_mecanum_drive_controller/transition_event",
            ),
        ],
        condition=UnlessCondition(use_sim),
    )

    # ========== 5. 启动控制器 spawner ==========
    # 在仿真模式下，控制器已经由 Gazebo 插件加载，只需要激活它们
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mecanum_drive_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
    )

    # IMU广播器
    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
    )
    
    # ========== 6. 延迟启动机制 ==========
    # 延迟启动 joint_state_broadcaster（给 controller_manager 初始化时间）
    delayed_joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner]
    )

    # 延迟启动 robot_controller（在 joint_state_broadcaster 之后）
    delayed_robot_controller_spawner = TimerAction(
        period=8.0,
        actions=[robot_controller_spawner]
    )

    # ========== 7. 错误处理 ==========
    def check_if_log_is_fatal(event):
        msg = event.text.decode().lower()
        if ("fatal" in msg or "failed" in msg) and "cyclonedds" not in msg:
            return EmitEvent(event=Shutdown(reason="Spawner failed"))

    spawn_monitor = RegisterEventHandler(
        OnProcessIO(
            target_action=joint_state_broadcaster_spawner,
            on_stderr=check_if_log_is_fatal,
        )
    )

    # ========== 8. 返回启动描述 ==========
    return LaunchDescription(
        [
            # 参数声明
            declare_namespace_arg,
            declare_use_sim_arg,

            # 启动节点
            spawn_monitor,
            control_node,
            delayed_joint_state_broadcaster_spawner,
            delayed_robot_controller_spawner,
        ]
    )

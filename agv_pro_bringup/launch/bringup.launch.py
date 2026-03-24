#!/usr/bin/env python3
"""
AGV Pro Bringup Launch File
Main entry point for hardware mode
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ========== 1. Define launch arguments ==========
    namespace = LaunchConfiguration("namespace", default="")
    robot_model = LaunchConfiguration("robot_model", default="agv_pro")

    # ========== 2. Declare launch arguments ==========
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable("ROBOT_MODEL_NAME", default_value="agv_pro"),
        description="Specify robot model",
        choices=["agv_pro"],
    )

    # ========== 3. Package paths ==========
    agv_pro_bringup = FindPackageShare("agv_pro_bringup")
    agv_pro_hardware = FindPackageShare("agv_pro_hardware_interfaces")
    controller_manager = FindPackageShare("controller_manager")
    agv_pro_description = FindPackageShare("agv_pro_description")
    agv_pro_controller = FindPackageShare("agv_pro_controller")

    # ========== 4. Launch robot state publisher ==========
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([agv_pro_description, "launch", "robot_state_publisher.launch.py"])
        ),
    )

    # ========== 5. Launch controller manager ==========
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([agv_pro_controller, "launch", "agv_pro_controller.launch.py"])
        ),
        launch_arguments={
            "use_sim": "False",
            "namespace": namespace,
        }.items(),
    )

    # ========== 6. System status info ==========
    status_info = LogInfo(
        msg=[
            "AGV Pro Hardware Mode Started\n",
            "\tNamespace: '", namespace, "'\n",
            "\tRobot Model: '", robot_model, "'\n",
        ]
    )

    # ========== 7. Return launch description ==========
    return LaunchDescription([
        # Argument declarations
        declare_namespace_arg,
        declare_robot_model_arg,

        # Global settings
        SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1"),
        SetEnvironmentVariable(name="ROS_LOCALHOST_ONLY", value="0"),

        # Launch components
        status_info,
        robot_state_publisher,
        
        # Delayed launch for controller
        TimerAction(
            period=2.0,
            actions=[controller_launch]
        ),
    ])
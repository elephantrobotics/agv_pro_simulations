#!/usr/bin/env python3
"""
AGV Pro Simulation Launch File
总入口启动文件，整合 Gazebo、控制器和模型生成
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import SetParameter, SetRemap, Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ========== 1. 定义启动参数 ==========
    config_dir = LaunchConfiguration("config_dir", default="")
    rviz = LaunchConfiguration("rviz", default="False")
    use_sim = LaunchConfiguration("use_sim", default="True")

    # ========== 2. 声明启动参数 ==========
    declare_config_dir_arg = DeclareLaunchArgument(
        "config_dir",
        default_value="",
        description="Path to the common configuration directory.",
    )

    declare_rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="False",
        description="Run RViz simultaneously.",
        choices=["True", "False"],
    )

    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="True",
        description="Whether to use simulation.",
        choices=["True", "False"],
    )

    # ========== 3. 构建世界文件路径 ==========
    world_path = PathJoinSubstitution([
        FindPackageShare("agv_pro_gazebo"),
        "worlds",
        "agv_pro_world.world"
    ])
    
    # ========== 4. 启动 Gazebo 仿真环境 ==========
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            ])
        ),
        launch_arguments={
            "gz_args": ["-r ", world_path],  # 运行自定义世界
            "gz_log_level": "1",  # 减少日志输出
            "step_size": "0.01",
        }.items(),
    )

    # ========== 6. 启动模型生成和控制器 ==========
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("agv_pro_gazebo"),
                "launch",
                "spawn_agv_pro.launch.py",
            ])
        ),
        launch_arguments={
            "use_sim": use_sim,
        }.items(),
    )

    # ========== 7. 启动 RViz (可选) ==========
    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(rviz),
    )

    # ========== 8. 返回启动描述 ==========
    return LaunchDescription([
        # 参数声明
        declare_config_dir_arg,
        declare_rviz_arg,
        declare_use_sim_arg,

        # 全局设置
        SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1"),
        SetRemap("/diagnostics", "diagnostics"),
        SetRemap("/tf", "tf"),
        SetRemap("/tf_static", "tf_static"),
        SetParameter(name="use_sim_time", value=True),

        # 启动组件
        gz_sim,
        
        # 延迟启动机器人
        TimerAction(
            period=3.0,
            actions=[spawn_robot]
        ),
        
        rviz_launch,
    ])
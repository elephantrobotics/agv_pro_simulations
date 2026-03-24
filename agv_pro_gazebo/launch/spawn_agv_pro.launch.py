#!/usr/bin/env python3
"""
AGV Pro Spawn Launch File
职责：发布 robot_description、在 Gazebo 中生成机器人模型、启动 ROS-Gazebo 桥接
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
    RegisterEventHandler,
    EmitEvent,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessIO
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    PythonExpression,
    FindExecutable,
)
from launch_ros.actions import (
    Node,
    PushRosNamespace,
    SetParameter,
    SetRemap,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ========== 1. 定义启动参数 ==========
    namespace = LaunchConfiguration("namespace", default="")
    robot_model = LaunchConfiguration("robot_model", default="agv_pro")
    x = LaunchConfiguration("x", default="0.0")
    y = LaunchConfiguration("y", default="0.0")
    z = LaunchConfiguration("z", default="0.01")
    roll = LaunchConfiguration("roll", default="0.0")
    pitch = LaunchConfiguration("pitch", default="0.0")
    yaw = LaunchConfiguration("yaw", default="0.0")
    use_sim = LaunchConfiguration("use_sim", default="True")

    # ========== 2. 声明启动参数 ==========
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Add namespace to all launched nodes.",
    )

    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value="agv_pro",
        description="Specify robot model",
        choices=["agv_pro"],
    )

    declare_x_arg = DeclareLaunchArgument(
        "x", default_value="0.0", description="Initial x position"
    )

    declare_y_arg = DeclareLaunchArgument(
        "y", default_value="-0.4", description="Initial y position"
    )

    declare_z_arg = DeclareLaunchArgument(
        "z", default_value="0.02", description="Initial z position"
    )

    declare_roll_arg = DeclareLaunchArgument(
        "roll", default_value="0.0", description="Initial roll orientation"
    )

    declare_pitch_arg = DeclareLaunchArgument(
        "pitch", default_value="0.0", description="Initial pitch orientation"
    )

    declare_yaw_arg = DeclareLaunchArgument(
        "yaw", default_value="0.0", description="Initial yaw orientation"
    )

    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="True",
        description="Whether to use simulation.",
        choices=["True", "False"],
    )

    # ========== 3. 处理命名空间 ==========
    ns = PythonExpression(
        ["'", namespace, "' + '/' if '", namespace, "' else ''"]
    )
    robot_name = PythonExpression(
        ["'agv_pro'", " if '", namespace, "' == '' ", "else ", "'", namespace, "'"]
    )

    # ========== 4. 发布 robot_description ==========
    # 添加控制器配置文件路径
    controller_config = PathJoinSubstitution([
        FindPackageShare("agv_pro_controller"),
        "config",
        "agv_control.yaml"
    ])

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("agv_pro_description"),
                "urdf",
                "agv_pro.urdf.xacro"
            ]),
            " ",
            "use_sim:=", use_sim,
            " ",
            "controller_config:=", controller_config,
        ]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        parameters=[{"robot_description": robot_description_content,
                     "publish_frequency": 50.0,}],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    # ========== 5. 在 Gazebo 中生成模型 ==========
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", robot_name,
            "-allow_renaming", "true",
            "-topic", "robot_description",
            "-x", x,
            "-y", y,
            "-z", z,
            "-R", roll,
            "-P", pitch,
            "-Y", yaw,
        ],
        output="screen",
    )

    # ========== 6. 启动 ROS-Gazebo 桥接 ==========
    gz_bridge_config = PathJoinSubstitution(
        [FindPackageShare("agv_pro_gazebo"), "config", "agv_pro_bridge.yaml"]
    )
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="agv_pro_gz_bridge",
        output="screen",
        parameters=[
            {"config_file": gz_bridge_config},
        ],
        condition=IfCondition(use_sim),
    )

    # ========== 7. 启动控制器 ==========
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("agv_pro_controller"),
                "launch",
                "agv_pro_controller.launch.py",
            ])
        ),
        launch_arguments={
            "use_sim": use_sim,
            "namespace": namespace,
        }.items(),
    )

    # ========== 8. 延迟启动机制 ==========
    # 延迟生成模型（给 Gazebo 初始化时间）
    delayed_spawn = TimerAction(
        period=2.0,
        actions=[gz_spawn_entity]
    )

    # 延迟启动控制器（给模型生成和初始化时间）
    delayed_controller_launch = TimerAction(
        period=7.0,  # 模型生成后5秒启动控制器
        actions=[controller_launch]
    )

    # ========== 9. 错误处理 ==========
    def check_if_log_is_fatal(event):
        msg = event.text.decode().lower()
        if ("fatal" in msg or "failed" in msg) and "cyclonedds" not in msg:
            return EmitEvent(event=Shutdown(reason="Spawner failed"))

    spawn_monitor = RegisterEventHandler(
        OnProcessIO(
            target_action=gz_spawn_entity,
            on_stderr=check_if_log_is_fatal,
        )
    )

    # ========== 10. 欢迎信息 ==========
    welcome_msg = LogInfo(
        msg=[
            "Spawning AGV Pro\n",
            "\tNamespace: '", namespace, "'\n",
            "\tInitial pose: (", x, ", ", y, ", ", z, ", ",
            roll, ", ", pitch, ", ", yaw, ")",
        ]
    )

    # ========== 11. 返回启动描述 ==========
    return LaunchDescription([
        # 参数声明
        declare_namespace_arg,
        declare_robot_model_arg,
        declare_x_arg,
        declare_y_arg,
        declare_z_arg,
        declare_roll_arg,
        declare_pitch_arg,
        declare_yaw_arg,
        declare_use_sim_arg,

        # 全局设置
        PushRosNamespace(namespace),
        SetRemap("/diagnostics", "diagnostics"),
        SetRemap("/tf", "tf"),
        SetRemap("/tf_static", "tf_static"),
        SetParameter(name="use_sim_time", value=True),

        # 启动节点 - 调整顺序
        welcome_msg,
        spawn_monitor,
        TimerAction(
        period=1.0,
        actions=[gz_bridge]
        ),
        robot_state_pub_node,
        delayed_spawn,      # 先生成模型
        delayed_controller_launch,  # 后启动控制器
    ])

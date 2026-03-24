import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('agv_pro_fake_node'),
            'param',
            'agv_pro.yaml'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 正确的URDF文件路径
    urdf = os.path.join(
        current_dir,
        'agv_pro.urdf')

    # RViz配置文件路径（可使用turtlebot3的配置或创建自己的）
    rviz_config_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'rviz',
        'tb3_gazebo.rviz'
    )
    
    return LaunchDescription([
        LogInfo(msg=['Execute AGV Pro Fake Node!!']),

        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Specifying parameter direction'),
            
        # 添加RViz启动节点
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),

        Node(
            package='agv_pro_fake_node',
            executable='agv_pro_fake_node',
            parameters=[param_dir],
            output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['/home/ak/agv_pro_simulations-humble/src/agv_pro_fake_node/urdf/agv_pro.urdf']),  # 替换为您的URDF文件路径
    ])

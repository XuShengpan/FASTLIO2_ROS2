import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # 获取 config.yaml 的路径
    config_path = PathJoinSubstitution(
        [FindPackageShare("save_map_online"), "config", "config.yaml"]
    )

    return LaunchDescription([
        Node(
            package='save_map_online',
            executable='save_map_online_node',
            name='save_map_online_node',
            output='screen',
            # 【关键修改】：把 config_path 作为一个普通的字符串参数传给节点
            parameters=[{"config_path": config_path}]
        )
    ])
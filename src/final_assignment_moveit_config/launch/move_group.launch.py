import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. 声明 use_sim_time 参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true',
        description='Use simulation clock if true')

    # 2. 加载 MoveIt 配置
    moveit_config = MoveItConfigsBuilder("my_mobile_manipulator", package_name="final_assignment_moveit_config").to_moveit_configs()

    # 3. 定义 move_group 节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time}, # <--- 关键！强制 MoveIt 使用仿真时间
            {'trajectory_execution.allowed_execution_duration_scaling': 2.0}, # 允许执行时间稍微宽松一点
            {'publish_robot_description_semantic': True},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        move_group_node,
    ])

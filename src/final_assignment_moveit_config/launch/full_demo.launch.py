import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 2. 获取包路径
    pkg_gazebo = get_package_share_directory('final_assignment_pkg')
    pkg_moveit = get_package_share_directory('final_assignment_moveit_config')

    # 3. 包含 Gazebo 启动文件 (包含机器人生成和控制器加载)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 4. 包含 MoveIt 核心 (move_group) 启动文件
    moveit_config = MoveItConfigsBuilder("my_mobile_manipulator", package_name="final_assignment_moveit_config").to_moveit_configs()
    
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # 这里 MoveItConfigsBuilder 会自动处理参数，通常不需要手动传过多参数
        }.items()
    )

    # 5. 包含 RViz 启动文件
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, 'launch', 'moveit_rviz.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        gazebo_launch,
        move_group_launch,
        rviz_launch
    ])

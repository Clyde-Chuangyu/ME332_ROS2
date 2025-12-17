
首次运行前，或修改代码后，必须重新编译。

打开一个新终端，执行以下命令：
cd ~/proj_ws
colcon build --symlink-install
source install/setup.bash

以后每打开一个新的终端窗口，都必须先执行：
source ~/proj_ws/install/setup.bash

2. 启动步骤 (Launch Steps)


[终端 1] 启动 Gazebo 仿真环境 (物理世界)
---------------------------------------
source ~/proj_ws/install/setup.bash
ros2 launch final_assignment_pkg gazebo.launch.py

> 检查点：等待 Gazebo 窗口打开，确认机器人和方块已加载，且终端无红字报错。

[终端 2] 启动 MoveIt 核心 (运动规划大脑)
---------------------------------------
source ~/proj_ws/install/setup.bash
ros2 launch final_assignment_moveit_config move_group.launch.py use_sim_time:=true

> 检查点：等待终端显示 "You can start planning now!"。

[终端 3] 启动 RViz (可视化界面)
---------------------------------------
source ~/proj_ws/install/setup.bash
ros2 launch final_assignment_moveit_config moveit_rviz.launch.py use_sim_time:=true

> RViz 设置提示：
> 1. 左侧 Global Options -> Fixed Frame 修改为 "odom" 或 "base_footprint"。
> 2. 确保左下角已添加 "MotionPlanning" 组件。

3. 运行抓取搬运任务 (Run Mission)


[终端 4] 运行 Python 控制脚本
----------------------------
source ~/proj_ws/install/setup.bash
python3 ~/proj_ws/carry_mission.py


4. 控制底盘移动 (Teleop Control)

[终端 5] 启动键盘控制节点
------------------------
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

> 操作说明：
> - 确保键盘输入焦点在这个终端窗口上。
> - 按 'i' 前进，',' 后退，'j' 左转，'l' 右转，'k' 停止。
> - 控制小车把方块运送到目标地点。

5. 完成任务 (Finish)
--------------------------------------------------------------
当小车到达目标地点后：

1. 切换回 [终端 4] (运行 simple_carry.py 的窗口)。
2. 按下键盘上的 【回车键 (Enter)】。

6.综合程序（gazebo+rviz2+moveit2）
ros2 launch final_assignment_moveit_config full_demo.launch.py


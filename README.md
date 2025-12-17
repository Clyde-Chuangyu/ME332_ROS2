# ME332_ROS2
# 1. 更新 rosdep 数据库
sudo rosdep init
rosdep update

# 2. 安装所有依赖
# --from-paths src: 扫描 src 文件夹下的所有 package.xml
# --ignore-src: 忽略 src 中已有的包，只安装系统缺少的
# -r: 如果出错继续安装
# -y: 自动确认
rosdep install --from-paths src --ignore-src -r -y

sudo apt install ros-humble-moveit

sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-gazebo-ros2-control

sudo apt install ros-humble-gazebo-dev
sudo apt install ros-humble-gazebo-msgs

sudo apt install ros-humble-xacro
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-tf2-ros
sudo apt install ros-humble-robot-state-publisher


# 将 <你的仓库地址> 替换为你实际的 GitHub 地址
git clone <你的GitHub仓库地址> .
cd ~/ros2_ws
# 更新 rosdep 数据库
sudo rosdep init
rosdep update

# 安装依赖
rosdep install --from-paths src --ignore-src -r -y
# 编译并创建符号链接
colcon build --symlink-install

# 刷新环境变量
source install/setup.bash

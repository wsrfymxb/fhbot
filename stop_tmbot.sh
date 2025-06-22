#!/bin/bash

# TurtleBot3 Manipulation 自动关闭脚本
# 关闭所有通过自动启动脚本启动的 ROS2 节点和组件

# 打印关闭信息
echo "开始关闭 TurtleBot3 Manipulation 系统..."

# 1. 关闭 Gazebo 仿真
echo "步骤1: 关闭 Gazebo 仿真..."
pkill -f "gazebo.launch.py"
pkill -f "gzserver"
pkill -f "gzclient"
echo "Gazebo 仿真已关闭"

# 2. 关闭底盘导航系统
echo "步骤2: 关闭底盘导航系统..."
pkill -f "navigation2_use_sim_time.launch.py"
pkill -f "nav2_"
pkill -f "amcl"
pkill -f "local_costmap"
echo "底盘导航系统已关闭"

# 3. 关闭机械臂运动规划
echo "步骤3: 关闭机械臂运动规划..."
pkill -f "moveit_core.launch.py"
pkill -f "move_group"
pkill -f "moveit_"
echo "机械臂运动规划已关闭"

# 4. 关闭所有相关的 xterm 终端窗口
echo "步骤4: 关闭所有终端窗口..."
pkill -f "xterm -title Gazebo仿真"
pkill -f "xterm -title 底盘导航"
pkill -f "xterm -title 机械臂运动规划"
echo "终端窗口已关闭"

# 5. 清理 ROS 相关进程
echo "步骤5: 清理 ROS 相关进程..."
pkill -f "ros2"
pkill -f "rviz2"
echo "ROS 相关进程已清理"

# 6. 确认关闭状态
echo "步骤6: 验证关闭状态..."
sleep 2  # 给进程终止留出时间

# 检查是否还有相关进程在运行
if pgrep -f "turtlebot3_manipulation" > /dev/null; then
    echo "警告: 检测到仍在运行的进程:"
    pgrep -lf "turtlebot3_manipulation"
    echo "请手动检查并关闭"
else
    echo "所有 TurtleBot3 Manipulation 进程已成功关闭"
fi

echo "关闭操作完成！"

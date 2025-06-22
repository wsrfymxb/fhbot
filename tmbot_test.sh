#!/bin/bash  
  
# TurtleBot3 Manipulation 自动启动脚本  
# 分别启动Gazebo、导航、机械臂运动规划和控制界面  
  
# 设置栈大小以避免段错误  
ulimit -s unlimited  
  
# 检查ROS2环境  
if [ -z "$ROS_DISTRO" ]; then  
    echo "错误: ROS2环境未设置"  
    exit 1  
fi  
  
echo "开始启动TurtleBot3 Manipulation系统..."  

# 1. 启动Gazebo仿真环境  
echo "步骤1: 启动Gazebo仿真环境..."  
xterm -title "Gazebo仿真" -hold -e "  
    export LD_LIBRARY_PATH=/usr/lib/wsl/lib
    export LIBVA_DRIVER_NAME=d3d12
    export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
    source /opt/ros/humble/setup.bash  
    source ~/workspace/install/setup.bash  
    ros2 launch turtlebot3_manipulation_bringup gazebo.launch.py start_rviz:=false  
" &  
  
# 等待Gazebo完全启动  
echo "等待Gazebo启动完成..."  
sleep 15  
  
# 2. 启动底盘导航系统  
echo "步骤2: 启动底盘导航系统..."  
xterm -title "底盘导航" -hold -e "  
    export LD_LIBRARY_PATH=/usr/lib/wsl/lib
    export LIBVA_DRIVER_NAME=d3d12
    export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
    source /opt/ros/humble/setup.bash  
    source ~/workspace/install/setup.bash  
    ros2 launch turtlebot3_manipulation_navigation2 navigation2_use_sim_time.launch.py start_rviz:=true  
" &  
  
# 等待导航系统启动  
echo "等待导航系统启动完成..."  
sleep 10  
  
# 3. 启动机械臂运动规划  
echo "步骤3: 启动机械臂运动规划..."  
xterm -title "机械臂运动规划" -hold -e "  
    export LD_LIBRARY_PATH=/usr/lib/wsl/lib
    export LIBVA_DRIVER_NAME=d3d12
    export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
    source /opt/ros/humble/setup.bash  
    source ~/workspace/install/setup.bash
    ulimit -s unlimited  
    ros2 launch turtlebot3_manipulation_moveit_config moveit_core.launch.py  
" &  
  
# 等待MoveIt启动  
echo "等待MoveIt启动完成..."  
sleep 10  

# ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"

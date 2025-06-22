#!/bin/bash                                                                                                             
                                                                                                                        
# TurtleBot3 Manipulation 无界面自动启动脚本                                                                           
# 直接后台启动Gazebo、导航、机械臂运动规划和控制                                                                        
                                                                                                                        
# 设置栈大小以避免段错误                                                                                                
ulimit -s unlimited                                                                                                     
                                                                                                                        
# 检查ROS2环境                                                                                                          
if [ -z "$ROS_DISTRO" ]; then                                                                                           
    echo "错误: ROS2环境未设置"                                                                                         
    exit 1                                                                                                              
fi                                                                                                                      
                                                                                                                        
echo "开始无界面启动TurtleBot3 Manipulation系统..."                                                                    
                                                                                                                        
# 设置环境变量（一次性）                                                                                                
export LD_LIBRARY_PATH=/usr/lib/wsl/lib                                                                             
export LIBVA_DRIVER_NAME=d3d12                                                                                      
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA                                                                       
source /opt/ros/humble/setup.bash                                                                                   
source ~/workspace/install/setup.bash                                                                   
                                                                                                                        
# 1. 启动Gazebo仿真环境                                                                                                 
echo "启动Gazebo仿真（后台模式）..."                                                                                     
ros2 launch turtlebot3_manipulation_bringup gazebo.launch.py start_rviz:=false > /dev/null 2>&1 &                                                                                    
GAZEBO_PID=$!                                                                                                           
                                                                                                                        
# 等待Gazebo完全启动                                                                                                    
echo "等待Gazebo初始化（约15秒）..."                                                                                    
sleep 15                                                                                                                
                                                                                                                        
# 2. 启动底盘导航系统                                                                                                   
echo "启动底盘导航系统（后台模式）..."                                                                                       
ros2 launch turtlebot3_manipulation_navigation2 navigation2_use_sim_time.launch.py start_rviz:=true > /dev/null 2>&1 &                                                                                   
NAV_PID=$!                                                                                                              
                                                                                                                        
# 等待导航系统启动                                                                                                      
echo "等待导航系统启动（约10秒）..."                                                                                          
sleep 10                                                                                                                
                                                                                                                        
# 3. 启动机械臂运动规划                                                                                                 
echo "启动机械臂运动规划（后台模式）..."                                                                                     
ros2 launch turtlebot3_manipulation_moveit_config moveit_core.launch.py > /dev/null 2>&1 &                                                                                    
MOVEIT_PID=$!                                                                                                           
                                                                                                                        
# 等待MoveIt启动                                                                                                        
echo "等待MoveIt初始化（约10秒）..."                                                                                            
sleep 10                                                                                                                
                                                                                                                        
# 显示后台进程状态                                                                                                      
echo -e "\n===== 系统启动完成 ====="                                                                                    
echo "Gazebo PID: $GAZEBO_PID"                                                                                          
echo "导航 PID:   $NAV_PID"                                                                                             
echo "MoveIt PID: $MOVEIT_PID"                                                                                          
echo -e "\n使用 'kill $GAZEBO_PID $NAV_PID $MOVEIT_PID' 关闭所有进程"                                                   
echo "或使用 'pkill -f \"ros2 launch\"' 终止所有ROS进程"                                                                
                                                                                                                        
# 保持脚本运行直到手动终止                                                                                              
echo -e "\n按 Ctrl+C 退出此脚本（后台进程将继续运行）"                                                                    
wait

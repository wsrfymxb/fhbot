#!/usr/bin/env python3  
  
import os  
from launch import LaunchDescription  
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction  
from launch.launch_description_sources import PythonLaunchDescriptionSource  
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution  
from launch_ros.actions import Node  
from launch_ros.substitutions import FindPackageShare  
  
def generate_launch_description():  
    # 启动参数  
    target_x1 = LaunchConfiguration('target_x1', default='1.5')  
    target_y1 = LaunchConfiguration('target_y1', default='1.0')  
    target_w1 = LaunchConfiguration('target_w1', default='1.0')
    target_x2 = LaunchConfiguration('target_x2', default='1.5')  
    target_y2 = LaunchConfiguration('target_y2', default='0.5')  
    target_w2 = LaunchConfiguration('target_w2', default='1.0')
      
    return LaunchDescription([  
        # 声明参数  
        DeclareLaunchArgument('target_x1', default_value='1.5'),  
        DeclareLaunchArgument('target_y1', default_value='1.0'),  
        DeclareLaunchArgument('target_w1', default_value='1.0'), 
        DeclareLaunchArgument('target_x2', default_value='1.5'),  
        DeclareLaunchArgument('target_y2', default_value='0.5'),  
        DeclareLaunchArgument('target_w2', default_value='1.0'), 
        TimerAction(  
            period=5.0,  
            actions=[  
                Node(  
                    package='turtlebot3_auto_manipulation',  
                    executable='auto_controller',  
                    name='auto_controller',  
                    parameters=[{  
                        'target_x1': target_x1,  
                        'target_y1': target_y1,  
                        'target_w1': target_w1,
                        'target_x2': target_x2,  
                        'target_y2': target_y2,  
                        'target_w2': target_w2,
                    }],  
                    output='screen'  
                )  
            ]  
        )  
    ])
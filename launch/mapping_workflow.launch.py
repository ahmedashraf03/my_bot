#!/usr/bin/env python3

import os
import time
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('my_bot')
    
    # Define paths
    world_path = os.path.join(pkg_dir, 'worlds', 'house.world')
    rviz_config = os.path.join(pkg_dir, 'config', 'house.rviz')
    mapper_params = os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')
    
    return LaunchDescription([
        # Launch simulation
        TimerAction(
            period=0.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'launch', 'my_bot', 'launch_sim.launch.py', f'world:={world_path}'],
                    name='simulation',
                    output='screen'
                ),
                LogInfo(msg='Starting simulation...')
            ]
        ),
        
        # Launch RViz after 6 seconds
        TimerAction(
            period=6.0,
            actions=[
                ExecuteProcess(
                    cmd=['rviz2', '-d', rviz_config],
                    name='rviz',
                    output='screen'
                ),
                LogInfo(msg='Starting RViz...')
            ]
        ),
        
        # Launch online mapper after 8 seconds
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'launch', 'my_bot', 'online_async_launch.py', 
                         f'params_file:={mapper_params}', 'use_sim_time:=true'],
                    name='mapper',
                    output='screen'
                ),
                LogInfo(msg='Starting online mapper...')
            ]
        ),
        
        # Launch navigation after 10 seconds
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'launch', 'my_bot', 'navigation_launch.py', 'use_sim_time:=true'],
                    name='navigation',
                    output='screen'
                ),
                LogInfo(msg='Starting navigation...')
            ]
        )
    ])

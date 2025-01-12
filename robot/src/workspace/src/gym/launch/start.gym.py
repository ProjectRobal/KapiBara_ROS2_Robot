import os
import logging
from launch import LaunchDescription,actions,substitutions

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node


def generate_launch_description():
    
    exec_name = LaunchConfiguration('exec')
    
    exec_name_arg = DeclareLaunchArgument(
        'exec',
        default_value='main.py'
    )
    
    logger = substitutions.LaunchConfiguration("log_level")
    
    gym = Node(
        package="gym",
        executable=exec_name,
        output='screen',
        emulate_tty=True,
        parameters=[{
            "use_sim_time":True
        }],
        arguments=['--ros-args', '--log-level', logger]
    )
    
    
    # Run the node
    return LaunchDescription([
        actions.DeclareLaunchArgument(
            "log_level",
            default_value=["info"],
            description="Logging level",
      ),
        exec_name_arg,
        gym
    ])



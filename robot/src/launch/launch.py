import os
import logging
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    
    

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'kapibara'

    launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
            get_package_share_directory(pkg_name),'launch','launch.kapibara.can.py')
        ]
    ))
    


    # Run the node
    return LaunchDescription([
        launch
    ])



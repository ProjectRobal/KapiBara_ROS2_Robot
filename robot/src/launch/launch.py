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
    # pkg_name = 'kapibara'

    # launch=IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #         get_package_share_directory(pkg_name),'launch','launch.gazebo.py')
    #     ]
    # ))
    
    pkg_name = 'gym'
    
    rviz = Node(package='rviz2', executable='rviz2',
                    arguments=[],
                    output='screen')

    launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
            get_package_share_directory(pkg_name),'launch','start.gym.py')
        ]
    ),
        launch_arguments={
            "exec":"main.py"
        }.items()
    )
    
    rqt=Node(package='rqt_image_view', executable='rqt_image_view',
                    arguments=[],
                    output='screen')


    # Run the node
    return LaunchDescription([
        launch
        #rviz
    ])



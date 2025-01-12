from launch.launch_service import LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

from billiard import Process

import os

def launch(launch_path:str,**kwargs)->Process:
    
    service = LaunchService()
    
    description = IncludeLaunchDescription(
        launch_description_source = PythonLaunchDescriptionSource([launch_path]),
        launch_arguments=kwargs.items(),
        )
    
    service.include_launch_description(description)
    
    process = Process(target=service.run)
    #process.start()
    
    return process
    

def launch_environment(env_name:str,**kwargs)->Process:
    
    launch_file = os.path.join(get_package_share_directory("gym"),"launch/launch."+env_name+".py")
    
    return launch(launch_file,**kwargs)

def launch_other(launch_filename:str,**kwargs)->Process:
    
    launch_file = os.path.join(get_package_share_directory("gym"),"launch",launch_filename+".py")
    
    return launch(launch_file,**kwargs)
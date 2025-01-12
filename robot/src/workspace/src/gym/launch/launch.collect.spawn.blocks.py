import os
import logging
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription,substitutions
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument,GroupAction,OpaqueFunction
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

from launch.actions import SetEnvironmentVariable

from launch.substitutions import LaunchConfiguration

'''
 
 Position for all collectables:

# 1:
 -1.86847, 2.12847, 0.5
# 2:
 0.109086, -1.9246, 0.5
# 3: 
 -3.04748, -3.18496, 0.5
# 4:
 -4.68853, 2.104755, 0.5

'''


launch_args =[
    DeclareLaunchArgument(
            "log_level",
            default_value=["debug"],
            description="Logging level",
    ),
    DeclareLaunchArgument(
        'points',
        default_value=''
    )
]

def launch_setup(context):
    logger = substitutions.LaunchConfiguration("log_level").perform(context)
    
    point_positions = LaunchConfiguration('points').perform(context)
    
    # points in format x1 y1,x2 y2,x3 y3,x4 y4     
    points = point_positions.split(",")    
    point_coords = []
    
    for point in points:
        coords = point.split()
        
        point_coords.append((float(coords[0]),float(coords[1])))
    
    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'gym'
    
    
    xacro_file = os.path.join(get_package_share_directory(pkg_name),'description/landmine.urdf.xacro')
    
    actions = []
    
    for i,(x,y) in enumerate(point_coords):
        
        spawn_mine = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=["-topic",f"/point{i}/robot_description","-entity","point"+str(i),"-timeout","240","-x",str(x),"-y",str(y),"-z","0.5"],
                    output='screen')
        actions.append(GroupAction(
            actions=[
                PushRosNamespace('point'+str(i)),
                spawn_mine
            ]
        ))

    return [*actions]

def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)
    
    # Run the node
    desc = LaunchDescription(launch_args)
    desc.add_action(opfunc)
    
    return desc



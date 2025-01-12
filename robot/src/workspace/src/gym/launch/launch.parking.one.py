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
 
 Position for good spots:

# 1:
 -2.48651, -0.042629, 0.4
# 2:
 -3.68679, -0.042629, 0.4

'''

POINTS_POSITIONS=[
    (-2.48651, -0.042629),
    (-3.68679, -0.042629)
    ]

launch_args =[
    DeclareLaunchArgument(
            "log_level",
            default_value=["debug"],
            description="Logging level",
    )
]

def launch_setup(context):
    logger = substitutions.LaunchConfiguration("log_level").perform(context)
    
    
    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'gym'
    
    robot_pkg_name = "kapibara"
    file_subpath = 'description/kapibara.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(robot_pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file,mappings={'sim_mode' : 'true'}).toxml()    
    
    xacro_file = os.path.join(get_package_share_directory(pkg_name),'description/landmine.urdf.xacro')
    landmine_description = xacro.process_file(xacro_file,mappings={'topic' : 'trigger1'}).toxml()
    
    gazebo_env = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(get_package_prefix("kapibara"), "share"))

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace='KapiBara',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}], # add other parameters here if required
        remappings=[
            ('/tf','/KapiBara/tf'),
            ('/tf_static','/KapiBara/tf_static')
        ]
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py',)]),
            launch_arguments={
                'world': os.path.join(get_package_share_directory(pkg_name),'worlds/world.sdf'),
                'params_file': os.path.join(get_package_share_directory(pkg_name),"config/gazebo.yaml"),
                'verbose': 'true'
                }.items()
        )
    
    spawn_maze = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=["-file",os.path.join(get_package_share_directory(pkg_name),"props/Parking_1/model.sdf"),"-entity","Parking","-timeout","240"],
                    output='screen')
    
    actions = []
    
    for i,(x,y) in enumerate(POINTS_POSITIONS):
        landmine_description = xacro.process_file(xacro_file,mappings={'topic' : 'trigger'+str(i), 'width' : '0.5', 'height' : '0.5',"color" : "Green"}).toxml()
        
        node_landmine_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': landmine_description,
        'use_sim_time': True}] # add other parameters here if required
        )
        
        spawn_mine = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=["-topic",f"/point{i}/robot_description","-entity","point"+str(i),"-timeout","240","-x",str(x),"-y",str(y),"-z","0.4"],
                    output='screen')
        actions.append(GroupAction(
            actions=[
                PushRosNamespace('point'+str(i)),
                node_landmine_state_publisher,
                spawn_mine
            ]
        ))

    return [node_robot_state_publisher,gazebo,spawn_maze,*actions]

def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)
    
    # Run the node
    desc = LaunchDescription(launch_args)
    desc.add_action(opfunc)
    
    return desc



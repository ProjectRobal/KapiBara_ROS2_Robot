import os
import logging
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription,substitutions
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument,GroupAction,OpaqueFunction,TimerAction
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

from launch.actions import SetEnvironmentVariable

from launch.substitutions import LaunchConfiguration

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
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace="KapiBara",
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}], # add other parameters here if required
        remappings=[
            ('/tf','/KapiBara/tf'),
            ('/tf_static','/KapiBara/tf_static')
        ]
    )

    # Use xacro to process the file    
    xacro_file_ = os.path.join(get_package_share_directory(pkg_name),'description/mouse.urdf.xacro')
    
    gazebo_env = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(get_package_prefix("kapibara"), "share"))

    
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
                    arguments=["-file",os.path.join(get_package_share_directory(pkg_name),"props/Predator1/model.sdf"),"-entity","Maze","-timeout","240"],
                    output='screen')
    
    actions = []
    
    mouse_description = xacro.process_file(xacro_file_,mappings={'name' : 'mouse'}).toxml()
    
    mouse_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[{'robot_description': mouse_description,
    'use_sim_time': True}], # add other parameters here if required
    namespace="mouse",
    remappings=[
            ('/tf','/mouse/tf'),
            ('/tf_static','/mouse/tf_static')
        ]
    )
    
    spawn_mouse = Node(package='gazebo_ros', executable='spawn_entity.py',
                arguments=["-topic","/mouse/robot_description","-robot_namespace","mouse","-entity","stefan","-timeout","240"],
                namespace="mouse",
                output='screen')
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="mouse",
        arguments=["mouse_motors",'--controller-manager-timeout','240','--ros-args']
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="mouse",
        arguments=["mouse_joint_state",'--controller-manager-timeout','240','--ros-args']
    )
    
    actions.append(GroupAction(
        actions=[
            mouse_state_publisher,
            spawn_mouse,
            diff_drive_spawner,
            joint_broad_spawner
        ]
    ))

    return [gazebo,node_robot_state_publisher,spawn_maze,*actions]

def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)
    
    # Run the node
    desc = LaunchDescription(launch_args)
    desc.add_action(opfunc)
    
    return desc



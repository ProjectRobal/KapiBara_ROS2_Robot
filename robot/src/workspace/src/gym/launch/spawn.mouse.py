import os

from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    
    gazebo_env = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(get_package_prefix("kapibara"), "share"))
    
    robot_name = LaunchConfiguration('name')
    
    position_x = LaunchConfiguration('x')
    position_y = LaunchConfiguration('y')
    position_z = LaunchConfiguration('z')
    
    rot_roll = LaunchConfiguration('roll')
    rot_pitch = LaunchConfiguration('pitch')
    rot_yaw = LaunchConfiguration('yaw')
    
    robot_name_arg = DeclareLaunchArgument(
        'name',
        default_value='mouse'
    )
    
    position_x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0'
    )
    
    position_y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0'
    )
    
    position_z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.0'
    )
    
    position_roll_arg = DeclareLaunchArgument(
        'roll',
        default_value='0.0'
    )
    
    position_pitch_arg = DeclareLaunchArgument(
        'pitch',
        default_value='0.0'
    )
    
    position_yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0'
    )
   
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=["-topic","/robot_description","-entity","kapibara","-timeout","240","-x",position_x,"-y",position_y,"-z",position_z,"-R",rot_roll,"-P",rot_pitch,"-Y",rot_yaw],
                    output='screen')
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_name+"_motors",'--controller-manager-timeout','240','--ros-args']
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_name+"joint_state",'--controller-manager-timeout','240','--ros-args']
    )
    
    # Run the node
    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value=["debug"],
            description="Logging level",
      ),
        robot_name_arg,
        position_x_arg,
        position_y_arg,
        position_z_arg,
        position_roll_arg,
        position_pitch_arg,
        position_yaw_arg,
        spawn,
        diff_drive_spawner,
        joint_broad_spawner
    ])



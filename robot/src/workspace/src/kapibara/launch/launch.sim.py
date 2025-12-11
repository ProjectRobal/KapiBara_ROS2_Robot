import os
import logging
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

from launch.actions import SetEnvironmentVariable

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'kapibara'
    file_subpath = 'description/kapibara.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file,mappings={'sim_mode' : 'true','robot_name' : 'KapiBara'}).toxml()

    gazebo_env = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(get_package_prefix("kapibara"), "share"))
    
    orb_wrapper_pkg = get_package_share_directory('orb_slam3_ros2_wrapper')

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace = 'KapiBara',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py',)]),
            launch_arguments={
                'world': '/app/src/rviz/playground.sdf',
                'params_file': os.path.join(get_package_share_directory(pkg_name),"config/gazebo.yaml"),
                }.items()
        )

    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=["-topic","/KapiBara/robot_description","-entity","kapibara","-timeout","240","-z","1"],
                    output='screen')
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="KapiBara",
        arguments=["motors",'--controller-manager-timeout','240'],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="KapiBara",
        arguments=["joint_broad",'--controller-manager-timeout','240'],
    )
    
    ears_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="KapiBara",
        arguments=["ears_controller",'--controller-manager-timeout','240'],
    )

    emotions = Node(
        package="emotion_estimer",
        executable="estimator.py",
        namespace="KapiBara",
        parameters=[{
            "sim":True
        }]
    )
    
    mind = Node(
        package="kapibara_mind",
        executable="mind",
        namespace="KapiBara",
        parameters=[
            {
                "max_linear_speed":0.4,
                "max_angular_speed":2.5,
                "angular_p":4.0,
                "angular_i":2.0
            }
        ]
    )
    
    mic = Node(
        package="microphone",
        executable="mic.py",
        namespace = 'KapiBara',
        arguments=[],
        parameters=[{"channels":2,"sample_rate":44100,"chunk_size":4096,"device_id":5}],
        output='screen'
    )
    
    mqtt_bridge = Node(
        package="mqtt_bridge",
        executable="mqtt_bridge",
        parameters=[{"stop_mind_enable":False}],
        remappings=[
            ('microphone','/KapiBara/mic'),
        ],
        namespace=""
    )
    
    parameters=[{
            'use_sim_time': True,
          'frame_id':'KapiBara_base_link',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'odom_frame_id': 'KapiBara_odom',
          'publish_tf':True,
          'approx_sync':True}]
    
    remappings=[
          ('rgb/image', '/KapiBara/camera/image_raw'),
          ('rgb/camera_info', '/KapiBara/camera/camera_info'),
          ('depth/image', '/KapiBara/camera/depth/image_raw')]
    
    rtabmap_odom = Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings,
            namespace="KapiBara")

    rtabmap_slam = Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d'],
            namespace="KapiBara")

    rtabmap_viz = Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings,
            namespace="KapiBara")
    
    # Run the node
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        #rviz,
        spawn,
        # emotions,
        diff_drive_spawner,
        joint_broad_spawner,
        ears_controller_spawner,
        # mind,
        # mqtt_bridge,
        mic,
        # voice_assitant
        rtabmap_odom,
        rtabmap_slam,
        rtabmap_viz
    ])



import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    
    pkg_name = 'diffbot_gazebo'

    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_depth_cam= LaunchConfiguration('use_depth_cam')
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(pkg_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 
                                       'use_ros2_control': use_ros2_control,
                                       'use_depth_cam': use_depth_cam}.items()
    )

    # Use xacro to process the file
    gazebo_params_path = os.path.join(
                  get_package_share_directory(pkg_name),'config','gazebo_params.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path }.items()
        )
    
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity','my_bot',
                                   '-x', '0.0',
                                   '-y', '0.0',
                                   '-z', '0.0',],
                                   output='screen')

    diff_cont_spawner= Node(
        package='controller_manager',
        executable='spawner',
        arguments=["diff_cont"]
    )
    
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_broad"] 
    )
    # Run the node
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),
        DeclareLaunchArgument(
            'use_depth_cam',
            default_value='true',
            description='Use depth camera (RGBD) instead of monocular camera (RGB) if true'),
        rsp,
        gazebo,     
        spawn_entity,
        diff_cont_spawner,
        joint_broad_spawner
    ])
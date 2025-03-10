import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    
    pkg_name = 'diffbot_gazebo'

    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_depth_cam= LaunchConfiguration('use_depth_cam')
    gui= LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    world_file_default = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'turtlebot3_world.world')
    world = LaunchConfiguration('world')
    world_file = LaunchConfiguration('world_file')
    # world_file = os.path.join(get_package_share_directory(pkg_name),'worlds',world.perform())
    
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(pkg_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time, 
                                       'use_ros2_control': use_ros2_control,
                                       'use_depth_cam': use_depth_cam}.items()
    )


    gazebo_params_path = os.path.join(
                  get_package_share_directory(pkg_name),'config','gazebo_params.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path,
                              'gui': gui,
                              'world': world_file,
                              }.items(),
            
        )

    twist_mux_params = os.path.join(get_package_share_directory(pkg_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity','my_bot',
                                   '-x', '-2.0',
                                   '-y', '1',
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

    remappings=[('odometry/filtered', '/odom')]
    robot_localization_file_path = os.path.join(get_package_share_directory(pkg_name), 'config/ekf.yaml') 
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        remappings=remappings,
        parameters=[robot_localization_file_path, 
        {'use_sim_time': True}])
    

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(pkg_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
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
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Path to the world file to load in Gazebo'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='use sim time if true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='turtlebot3_world',
            description='world to load'
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value=[get_package_share_directory(pkg_name),'/worlds/',LaunchConfiguration('world'),'.world'],
            description='world to load'
        ),
        rsp,
        gazebo,
        twist_mux,
        spawn_entity,
        diff_cont_spawner,
        joint_broad_spawner,
        joystick
        # robot_localization
    ])
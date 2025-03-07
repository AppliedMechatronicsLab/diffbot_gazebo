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

    world_file_default = os.path.join(get_package_share_directory(pkg_name), 
                                      'worlds', 'empty_world.world')
    
    default_world = os.path.join(
        get_package_share_directory(pkg_name),
        'worlds',
        'empty.world'
        )    
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )
    
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(pkg_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time, 
                                       'use_ros2_control': use_ros2_control,
                                       'use_depth_cam': use_depth_cam}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
            launch_arguments={'gz_args': ['-r -v4 ', world],'on_exit_shutdown': 'true'
                              }.items(),     
        )

    twist_mux_params = os.path.join(get_package_share_directory(pkg_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )


    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name','my_bot',
                                   '-z', '0.1',],
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

# bridge 
    bridge_params = os.path.join(get_package_share_directory(pkg_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                '--ros-args',
                '-p',
                f'config_file:={bridge_params}'
            ],
        )
    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
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
        world_arg,
        rsp,
        gazebo,
        twist_mux,
        spawn_entity,
        diff_cont_spawner,
        joint_broad_spawner,
        joystick,
        # robot_localization
        ros_gz_bridge,
        ros_gz_image_bridge
    ])
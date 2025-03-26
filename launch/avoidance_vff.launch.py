from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    vff_avoidance_cmd = Node(
        package='diffbot_gazebo',
        executable='avoidance_vff',
        parameters=[{'use_sim_time': True}],

        remappings=[
            ('input_scan', '/scan'),
            ('output_vel', '/diff_cont/cmd_vel_unstamped')
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(vff_avoidance_cmd)
    return ld
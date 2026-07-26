import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory(
        'husky_tunnel_bringup'
    )

    params_file = os.path.join(
        package_share,
        'config',
        'nav2_params.yaml'
    )

    map_file = os.path.join(
        package_share,
        'maps',
        'tunnel_nav2.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    tf_remappings = [
        ('/tf', '/a200_0000/tf'),
        ('/tf_static', '/a200_0000/tf_static'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                params_file,
                {
                    'yaml_filename': map_file,
                    'use_sim_time': use_sim_time,
                },
            ],
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
            remappings=tf_remappings + [
                (
                    'scan',
                    '/a200_0000/sensors/lidar3d_0/scan'
                ),
            ],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
        ),
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory(
        'husky_tunnel_bringup'
    )

    localization_file = os.path.join(
        package_share,
        'launch',
        'tunnel_localization.launch.py'
    )

    navigation_file = os.path.join(
        package_share,
        'launch',
        'tunnel_navigation.launch.py'
    )

    params_file = os.path.join(
        package_share,
        'config',
        'nav2_params.yaml'
    )

    rviz_file = os.path.join(
        package_share,
        'config',
        'tunnel_nav2.rviz'
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_file),
        launch_arguments={
            'use_sim_time': 'true',
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_file),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': params_file,
            'use_composition': 'False',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='tunnel_nav2_rviz',
        output='screen',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/tf', '/a200_0000/tf'),
            ('/tf_static', '/a200_0000/tf_static'),
        ],
    )

    return LaunchDescription([
        localization,
        rviz,
        TimerAction(
            period=3.0,
            actions=[navigation],
        ),
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_share = get_package_share_directory(
        "husky_tunnel_bringup"
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share,
                "launch",
                "tunnel_sim.launch.py",
            )
        )
    )

    lio_sam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share,
                "launch",
                "tunnel_lio_sam_full.launch.py",
            )
        )
    )

    return LaunchDescription([
        simulation,

        # Allow Gazebo, sensors and TF to initialize first.
        TimerAction(
            period=12.0,
            actions=[lio_sam],
        ),
    ])

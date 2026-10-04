"""Deprecated: use slam_ekf.launch.py or slam_mapping.launch.py instead.

Kept so old instructions don't break; forwards to slam_ekf.launch.py
in mapping mode.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('myrobot_controller')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'slam_ekf.launch.py')
            ),
            launch_arguments=[
                ('mode', 'mapping'),
                ('use_sim_time', 'true'),
                ('headless', 'false'),
            ],
        )
    ])

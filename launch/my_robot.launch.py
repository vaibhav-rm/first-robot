"""Launch robot in an empty world (Gazebo Sim / Harmonic).

Wrapper around slam_ekf.launch.py for backwards compatibility.
For the obstacle + ArUco world see my_robot.launchworld.py.
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

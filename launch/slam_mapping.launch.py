"""Standalone SLAM Toolbox launcher with mapping/localization switch.

This satisfies the task requirement: "Create your own launch file which has
parameter whether to run in mapping mode or localization mode".

Usage:
  ros2 launch myrobot_controller slam_mapping.launch.py mode:=mapping
  ros2 launch myrobot_controller slam_mapping.launch.py mode:=localization map_file:=/path/to/my_map

NOTE: robot_state_publisher already publishes base_link -> lidar_link /
depth_camera_link / imu_link / gps_link TFs from the URDF, so no extra static
TF is needed here. (The old base_link -> lidar static TF was wrong and is removed.)
Use this together with slam_ekf.launch.py in headless mode, or with your own
Gazebo bringup.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('myrobot_controller')

    mode = LaunchConfiguration('mode', default='mapping')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    default_map = os.path.join(pkg_share, 'maps', 'my_map')
    map_file = LaunchConfiguration('map_file', default=default_map)

    slam_mapping_params = os.path.join(pkg_share, 'config', 'mapper_params_mapping.yaml')
    slam_localization_params = os.path.join(pkg_share, 'config', 'mapper_params_localization.yaml')

    slam_toolbox_mapping = Node(
        condition=IfCondition(PythonExpression(["'", mode, "' == 'mapping'"])),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_mapping_params, {'use_sim_time': use_sim_time}],
    )

    slam_toolbox_localization = Node(
        condition=IfCondition(PythonExpression(["'", mode, "' == 'localization'"])),
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_localization_params,
            {'use_sim_time': use_sim_time},
            {'map_file_name': map_file},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping',
                              description='mapping or localization'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_file', default_value=default_map,
                              description='Map path without extension (localization mode)'),
        slam_toolbox_mapping,
        slam_toolbox_localization,
    ])

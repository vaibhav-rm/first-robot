from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # 🔹 Static TF: base_link -> lidar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
            arguments=[
                '0', '0', '0.25',   # xyz
                '0', '0', '0',      # rpy
                'base_link',
                'lidar'
            ]
        ),

        # 🔹 SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': True,

                # Mapping
                'mode': 'mapping',

                # ✅ CORRECT scan topic
                'scan_topic': '/scan',

                # Frames
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',

                # Map
                'resolution': 0.05,
                'map_update_interval': 2.0,
                'publish_map': True,

                # Stability
                'throttle_scans': 1,
                'minimum_time_interval': 0.1
            }]
        )
    ])


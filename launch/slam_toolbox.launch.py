from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    TimerAction,
    RegisterEventHandler
)
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ---------------- PATHS ----------------
    pkg_path = get_package_share_directory('myrobot_controller')

    world_path = os.path.join(
        pkg_path, 'worlds', 'simple_obstacles.world'
    )

    urdf_path = os.path.join(
        pkg_path, 'urdf', 'myrobot.urdf'
    )

    rviz_config = os.path.join(
        pkg_path, 'rviz', 'slam.rviz'
    )

    slam_params = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'config',
        'mapper_params_online_async.yaml'
    )

    # ---------------- GAZEBO ----------------
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            world_path,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    # ---------------- ROBOT STATE PUBLISHER ----------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': open(urdf_path).read()
        }]
    )

    # ---------------- SPAWN ROBOT ----------------
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'my_robot',
                    '-topic', 'robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.3'
                ],
                output='screen'
            )
        ]
    )

    # ---------------- SLAM TOOLBOX (START EARLY) ----------------
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {'use_sim_time': True}
        ]
    )

    # ---------------- RVIZ (START AFTER SLAM) ----------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    start_rviz_after_slam = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_toolbox,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[rviz]
                )
            ]
        )
    )

    # ---------------- FINAL ----------------
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        slam_toolbox,
        start_rviz_after_slam
    ])


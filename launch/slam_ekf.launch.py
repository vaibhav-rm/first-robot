"""Canonical full-system launch: Gazebo Sim + robot + bridges + EKF (local+global) + SLAM Toolbox.

Usage:
  Mapping (build the map with teleop):
    ros2 launch myrobot_controller slam_ekf.launch.py mode:=mapping
    # drive with teleop, then in another terminal:
    ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map --ros-args -p use_sim_time:=true
    # copy my_map.pgm/.yaml into <pkg>/maps/ and commit

  Localization (reuse saved map, check no drift):
    ros2 launch myrobot_controller slam_ekf.launch.py mode:=localization map_file:=<pkg>/maps/my_map

Topics bridged from Gazebo Sim (gz -> ROS 2):
  /scan, /imu/data, /gps/fix, /wheel/odom, /cmd_vel (ROS->gz),
  /camera/image_raw, /camera/depth_image, /camera/camera_info, /camera/points, /clock
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ================= PATHS =================
    pkg_share = get_package_share_directory('myrobot_controller')

    urdf_path = os.path.join(pkg_share, 'urdf', 'myrobot.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'simple_obstacles.world')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    # EKF configs
    ekf_local_config = os.path.join(pkg_share, 'config', 'ekf_local.yaml')
    ekf_global_config = os.path.join(pkg_share, 'config', 'ekf_global.yaml')
    navsat_config = os.path.join(pkg_share, 'config', 'navsat_transform.yaml')

    # SLAM Toolbox configs
    slam_mapping_params = os.path.join(pkg_share, 'config', 'mapper_params_mapping.yaml')
    slam_localization_params = os.path.join(pkg_share, 'config', 'mapper_params_localization.yaml')

    # ================= ARGUMENTS =================
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    mode = LaunchConfiguration('mode', default='mapping')
    headless = LaunchConfiguration('headless', default='false')

    # SLAM Toolbox localization expects the map path WITHOUT extension.
    default_map = os.path.join(pkg_share, 'maps', 'my_map')
    map_file = LaunchConfiguration('map_file', default=default_map)

    # ================= NODES =================

    # 1. Gazebo Sim (Harmonic)
    gz_args = PythonExpression(["'-r -v 4 ' + ('-s ' if '", headless, "' == 'true' else '') + '", world_path, "'"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=[('gz_args', gz_args)]
    )

    # 2. Robot State Publisher (publishes URDF TFs)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(Command(['cat ', urdf_path]), value_type=str)
        }]
    )

    # 4. Spawn Robot in Gazebo from robot_description topic
    # Spawn from /robot_description topic (no URDF->SDF file conversion:
    # file conversion lumps fixed joints and can drop <plugin> blocks).
    spawn_robot = TimerAction(
        period=5.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'my_robot',
                '-topic', 'robot_description',
                '-x', '0.0', '-y', '0.0', '-z', '0.3'
            ],
            output='screen'
        )]
    )

    # 5. EKF Local Node: fuses wheel encoders (wheel/odom) + IMU -> odom -> base_footprint
    #    world_frame=odom => publishes odom->base_footprint TF (local / continuous odometry)
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_node',
        output='screen',
        parameters=[ekf_local_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', 'odometry/local')
        ]
    )

    # 6. NavSat Transform Node: converts raw GPS (/gps/fix) to local cartesian (/gps/odom)
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('gps/fix', '/gps/fix'),
            ('imu', '/imu/data'),
            ('odometry/filtered', 'odometry/global'),
            ('odometry/gps', '/gps/odom')
        ]
    )

    # 7. EKF Global Node: fuses odometry/local + GPS odom + IMU -> map -> odom TF
    #    world_frame=map => publishes map->odom TF (global localisation, corrects drift)
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',
        output='screen',
        parameters=[ekf_global_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', 'odometry/global')
        ]
    )

    # 8. SLAM Toolbox - Mapping Mode (mode:=mapping)
    slam_toolbox_mapping = Node(
        condition=IfCondition(PythonExpression(["'", mode, "' == 'mapping'"])),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_mapping_params,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 9. SLAM Toolbox - Localization Mode (mode:=localization, uses saved map)
    slam_toolbox_localization = Node(
        condition=IfCondition(PythonExpression(["'", mode, "' == 'localization'"])),
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_localization_params,
            {'use_sim_time': use_sim_time},
            {'map_file_name': map_file}
        ]
    )


    # SLAM Toolbox (Jazzy+) is lifecycle-managed: autostart configure+activate
    # so it subscribes /scan and publishes /map without manual transitions.
    slam_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',        condition=IfCondition(PythonExpression(["'", mode, "' != 'idle'"])),

        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': True,
                     'node_names': ['slam_toolbox']}])

    # 10. RViz2
    rviz2 = Node(
        condition=UnlessCondition(headless),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 11. Teleop Keyboard (run in a separate terminal if this fails in your setup:
    #     ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p use_sim_time:=true)
    teleop = Node(
        condition=UnlessCondition(headless),
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 12. Gazebo Sim -> ROS 2 bridge (gz transport to ROS topics)
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/wheel/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/simple_obstacles_world/model/my_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen'
    )


    # Canonical topic names: gz-sim publishes camera color on /camera and info
    # on /camera_info; relay to standard ROS names used by detector/Nav2.
    relay_image = Node(
        package='topic_tools', executable='relay',
        arguments=['/camera/image', '/camera/image_raw'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')
    relay_info = Node(
        package='topic_tools', executable='relay',
        arguments=['/camera_info', '/camera/camera_info'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')
    relay_joints = Node(
        package='topic_tools', executable='relay',
        arguments=['/world/simple_obstacles_world/model/my_robot/joint_state', '/joint_states'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    # Gazebo resource path so <uri>model://aruco_marker_N</uri> resolves
    model_path = os.path.join(pkg_share, 'models')
    set_env_action = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        model_path
    )

    return LaunchDescription([
        set_env_action,
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('mode', default_value='mapping',
                              description='SLAM Toolbox mode: mapping or localization'),
        DeclareLaunchArgument('map_file', default_value=default_map,
                              description='Map path without extension (localization mode)'),
        DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo headless (no GUI)'),

        gazebo,
        robot_state_publisher,
        spawn_robot,
        gz_bridge,
        relay_image,
        relay_info,
        relay_joints,

        ekf_local_node,
        navsat_transform_node,
        ekf_global_node,

        slam_toolbox_mapping,
        slam_toolbox_localization,
        slam_lifecycle_manager,

        rviz2,
        teleop
    ])

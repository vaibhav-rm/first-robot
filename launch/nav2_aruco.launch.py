"""Nav2 + ArUco mission using SLAM Toolbox localization (no AMCL).

Why not AMCL: in this headless container the sim runs at ~16% real-time,
which breaks AMCL's TF timing. SLAM Toolbox localization mode is more
tolerant and provides the same map->odom TF that Nav2 needs.

TF tree:
  map -> odom        (SLAM Toolbox localization)
  odom -> base_footprint (EKF local: wheel encoders + IMU)
  base_footprint -> base_link -> sensors (robot_state_publisher)

Run:
  ros2 launch myrobot_controller nav2_aruco.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('myrobot_controller')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    urdf_path = os.path.join(pkg_share, 'urdf', 'myrobot.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'simple_obstacles.world')
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    ekf_local_config = os.path.join(pkg_share, 'config', 'ekf_local.yaml')
    slam_localization_params = os.path.join(pkg_share, 'config', 'mapper_params_localization.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map_file', default=os.path.join(pkg_share, 'maps', 'my_map'))

    # ---- Gazebo Sim ----
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', '-r -v 4 ' + world_path)],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': ParameterValue(Command(['cat ', urdf_path]), value_type=str)}],
        output='screen')

    sdf_path = '/tmp/myrobot.sdf'
    convert_urdf = ExecuteProcess(
        cmd=['bash', '-c', f'gz sdf -p {urdf_path} > {sdf_path}'], output='screen')

    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'my_robot', '-topic', 'robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '0.3'],
        output='screen')

    gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/wheel/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen')

    # ---- EKF local only (odom -> base_footprint) ----
    ekf_local_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_local_node',
        parameters=[ekf_local_config, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/local')],
        output='screen')

    # ---- SLAM Toolbox localization (map -> odom + /map) ----
    slam_localization = Node(
        package='slam_toolbox', executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_localization_params,
            {'use_sim_time': use_sim_time},
            {'map_file_name': map_file},
        ],
        output='screen')

    # ---- Lifecycle manager for SLAM Toolbox (delayed to allow map load) ----
    from launch.actions import TimerAction
    slam_lifecycle_manager = TimerAction(
        period=20.0,
        actions=[Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'autostart': True,
                         'node_names': ['slam_toolbox']}])])

    # ---- Nav2 navigation (NO AMCL, NO map_server) ----
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_path,
            'autostart': 'true',
        }.items(),
    )

    # ---- RViz2 ----
    rviz2 = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    # ---- ArUco detector ----
    aruco_node = Node(
        package='myrobot_controller', executable='aruco_detector',
        name='aruco_detector',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    # ---- Waypoint navigator ----
    navigator_node = Node(
        package='myrobot_controller', executable='waypoint_navigator',
        name='waypoint_navigator',
        parameters=[{'use_sim_time': use_sim_time,
                     'run_static_mission': True,
                     'waypoints': [2.0, 1.2],
                     'halt_secs': 3.0}],
        output='screen')

    set_model_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_share, 'models'))

    return LaunchDescription([
        set_model_path,
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_file', default_value=os.path.join(pkg_share, 'maps', 'my_map')),
        gazebo, robot_state_publisher, convert_urdf, spawn_robot, gz_bridge,
        ekf_local_node,
        slam_localization,
        slam_lifecycle_manager,
        nav2_launch,
        rviz2,
        aruco_node,
        navigator_node,
    ])

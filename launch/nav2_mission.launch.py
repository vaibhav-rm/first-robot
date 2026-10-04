"""NAVIGATION WITH ARUCO task: chained marker missions with Nav2 + OpenCV.

Flow:
  1. Sim + EKF-local + Nav2 (same conflict-free layout as nav_mission.launch.py).
  2. waypoint_navigator starts with static goal in front of marker 1 (2.0, 1.2).
  3. aruco_detector sees id=1 -> draws border, publishes (-2.0,-0.8) to
     /Nav2_coordinates (coords of marker 2 encoded in marker 1).
  4. waypoint_navigator receives /Nav2_coordinates -> Nav2 goToPose -> 3s halt.
  5. Sees id=2 -> publishes (1.2,-2.2) -> navigate -> sees id=3 -> mission complete.
  6. Annotated images with borders: /aruco/annotated_image.

Prereq: saved map at maps/my_map.yaml (see nav_mission.launch.py docstring).
Run: ros2 launch myrobot_controller nav2_mission.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('myrobot_controller')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    urdf_path = os.path.join(pkg_share, 'urdf', 'myrobot.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'simple_obstacles.world')
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    ekf_local_config = os.path.join(pkg_share, 'config', 'ekf_local.yaml')
    default_map_yaml = os.path.join(pkg_share, 'maps', 'my_map.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml = LaunchConfiguration('map', default=default_map_yaml)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', '-r -v 4 ' + world_path)],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['cat ', urdf_path])}],
        output='screen')

    sdf_path = '/tmp/myrobot.sdf'
    convert_urdf = ExecuteProcess(
        cmd=['bash', '-c', f'gz sdf -p {urdf_path} > {sdf_path}'], output='screen')

    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'my_robot', '-file', sdf_path,
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
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen')

    ekf_local_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_local_node',
        parameters=[ekf_local_config, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/local')],
        output='screen')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_path,
            'map': map_yaml,
            'autostart': 'true',
        }.items(),
    )

    aruco_node = Node(
        package='myrobot_controller', executable='aruco_detector',
        name='aruco_detector',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    # Static mission delivers robot to marker 1; afterwards /Nav2_coordinates chain takes over.
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
        DeclareLaunchArgument('map', default_value=default_map_yaml),
        gazebo, robot_state_publisher, convert_urdf, spawn_robot, gz_bridge,
        ekf_local_node,
        nav2_launch,
        aruco_node,
        navigator_node,
    ])

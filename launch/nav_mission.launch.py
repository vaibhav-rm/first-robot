"""NAVIGATION task: autonomous waypoint following with Nav2.

Architecture (avoids TF conflicts):
  Gazebo Sim + robot_state_publisher + ros_gz_bridge + EKF-LOCAL only
  (ekf_local publishes odom->base_footprint)
  + Nav2 bringup (map_server + AMCL publishes map->odom, planner, controller)
  + waypoint_navigator.py (own python node: 3 waypoints, 3s halt each)

IMPORTANT: do NOT run slam_toolbox at the same time (it also publishes
map->odom and would fight AMCL). EKF-GLOBAL is disabled here for the same
reason; enable it only in slam_ekf.launch.py runs without AMCL.

Prereqs:
  1. Build a map first: ros2 launch myrobot_controller slam_ekf.launch.py mode:=mapping
     drive with teleop, save map to maps/my_map.yaml + maps/my_map.pgm
  2. Then: ros2 launch myrobot_controller nav_mission.launch.py
  3. Give extra waypoints in RViz ("Nav2 Goal") or via param `waypoints`.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable, TimerAction
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
    default_map_yaml = os.path.join(pkg_share, 'maps', 'my_map.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml = LaunchConfiguration('map', default=default_map_yaml)

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

    # Spawn from /robot_description topic (no URDF->SDF file conversion:
    # file conversion lumps fixed joints and can drop <plugin> blocks).
    spawn_robot = TimerAction(
        period=5.0,
        actions=[Node(
            package='ros_gz_sim', executable='create',
            arguments=['-name', 'my_robot', '-topic', 'robot_description',
                       '-x', '0.0', '-y', '0.0', '-z', '0.3'],
            output='screen')])

    gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
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
        output='screen')

    # ---- EKF local only (odom frame). Global EKF intentionally off (AMCL owns map->odom). ----
    ekf_local_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_local_node',
        parameters=[ekf_local_config, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/local')],
        output='screen')

    # ---- Nav2 (map_server + amcl + planner + controller + bt_navigator) ----
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_path,
            'map': map_yaml,
            'autostart': 'true',
        }.items(),
    )

    # ---- Own python waypoint node: 3 waypoints, 3s halt each ----
    navigator_node = Node(
        package='myrobot_controller', executable='waypoint_navigator',
        name='waypoint_navigator',
        parameters=[{'use_sim_time': use_sim_time,
                     'run_static_mission': True,
                     'waypoints': [2.0, 1.2, -2.0, -0.8, 1.2, -2.2],
                     'halt_secs': 3.0}],
        output='screen')


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

    set_model_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_share, 'models'))

    return LaunchDescription([
        set_model_path,
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map_yaml,
                              description='Full path to saved map YAML (from mapping run)'),
        gazebo, robot_state_publisher, spawn_robot, gz_bridge,
        relay_image, relay_info, relay_joints,
        ekf_local_node,
        nav2_launch,
        navigator_node,
    ])

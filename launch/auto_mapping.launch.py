"""Fully Autonomous Mapping: Gazebo + EKF local + SLAM Toolbox + Auto Explorer.

NO teleop needed - robot explores automatically for configurable duration.
Run: ros2 launch myrobot_controller auto_mapping.launch.py duration:=300
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('myrobot_controller')

    urdf_path = os.path.join(pkg_share, 'urdf', 'myrobot.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'simple_obstacles.world')
    ekf_local_config = os.path.join(pkg_share, 'config', 'ekf_local.yaml')
    slam_mapping_params = os.path.join(pkg_share, 'config', 'mapper_params_mapping.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    duration = LaunchConfiguration('duration', default='300')  # 5 minutes
    # RViz needs its own GL context, and the lidar is a gpu_lidar that traces
    # rays through the render engine. On this box (AMD APU, 2GB shared graphics
    # memory, Mesa software GL) running both makes the lidar run out of GPU
    # memory and start publishing all-inf ranges, which silently produces a
    # map with no walls in it. Default off; pass rviz:=true to watch it live.
    use_rviz = LaunchConfiguration('rviz', default='false')

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        # -s forces server-only. Without it Gazebo still tries to bring up the
        # GUI and dies on "could not connect to display" when run headless,
        # which silently stops the sim and leaves every scan empty. -r runs
        # paused=false, -v 4 is log verbosity.
        launch_arguments=[('gz_args', '-s -r -v 4 ' + world_path)],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': ParameterValue(Command(['cat ', urdf_path]), value_type=str)}],
        remappings=[('joint_states', 'joint_states')],
        output='screen')

    # Spawn robot from robot_description topic
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
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/wheel/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/world/simple_obstacles_world/model/my_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen')

    # EKF LOCAL ONLY (odom -> base_footprint)
    ekf_local_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_local_node',
        parameters=[ekf_local_config, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/local')],
        output='screen')

    # SLAM Toolbox MAPPING (async_slam_toolbox_node) - publishes map -> odom
    # NOTE: async_slam_toolbox_node is NOT lifecycle-managed (only localization node is)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_mapping_params, 
                    {'use_sim_time': use_sim_time,
                     'map_frame': 'map',
                     'odom_frame': 'odom',
                     'base_frame': 'base_footprint',
                     'scan_topic': 'scan'}],
    )

    # Topic relays for camera
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

    # Joint State Publisher - publishes wheel joint states for robot_state_publisher TF tree
    # Falls back to zero positions if Gazebo joint states aren't available
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'source_list': ['/world/simple_obstacles_world/model/my_robot/joint_state'],
                     'rate': 30.0}],
        output='screen')

    # SLAM Toolbox is a lifecycle node: it boots in 'unconfigured' and never
    # subscribes to /scan until configured + activated. Activate it after the
    # lidar bridge has data, otherwise no map is ever produced.
    slam_lifecycle_manager = TimerAction(
        period=8.0,
        actions=[Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='slam_lifecycle_manager',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'autostart': True,
                         'node_names': ['slam_toolbox'],
                         'bond_timeout': 0.0,
                         'transition_timeout': 30.0}])])

    # AUTO EXPLORER - delayed start to let SLAM initialize
    auto_explorer = TimerAction(
        period=15.0,  # Wait for SLAM to initialize and get first scans
        actions=[Node(
            package='myrobot_controller', executable='auto_explorer',
            name='auto_explorer',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'max_duration_sec': duration,
                         'forward_speed': 0.25,
                         'turn_speed': 0.7,
                         'obstacle_threshold': 0.75,
                         'geofence_radius': 4.5,
                         'scan_topic': 'scan',
                         'stuck_timeout': 4.0,
                         'stuck_distance_threshold': 0.08,
                         'rear_clear_threshold': 0.45,
                         # Arena bounds, matched to the physical perimeter
                         # walls in simple_obstacles.world (inner faces at
                         # x=+-3.1, y=-3.5..2.7). Inset by ~0.25m so the
                         # software bound is reached while the robot is still
                         # clear of the wall rather than after scraping it.
                         'x_min': -2.85,
                         'x_max': 2.85,
                         'y_min': -3.25,
                         'y_max': 2.45,
                         'idle_timeout': 30.0}])])

    # RViz2, started only once Gazebo's /clock is already flowing.
    # With use_sim_time, a node that initialises before the first /clock
    # message falls back to wall time; when the simulated clock then arrives
    # at t=0 RViz sees a ~2e9s backwards jump and resets itself. That reset
    # loop repeats until the GL context dies and the process exits, which is
    # what produces both the flood of "Detected jump back in time" warnings
    # and an empty "No map received" display.
    rviz2 = TimerAction(
        period=20.0,
        condition=IfCondition(use_rviz),
        actions=[Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')])

    set_model_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_share, 'models'))

    return LaunchDescription([
        set_model_path,
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('duration', default_value='300',
                              description='Exploration duration in seconds'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Launch RViz (needs its own GL context; competes with the gpu_lidar)'),
        gazebo, robot_state_publisher, spawn_robot, gz_bridge,
        relay_image, relay_info, relay_joints,
        ekf_local_node,
        slam_toolbox,
        slam_lifecycle_manager,
        auto_explorer,
        rviz2,
    ])